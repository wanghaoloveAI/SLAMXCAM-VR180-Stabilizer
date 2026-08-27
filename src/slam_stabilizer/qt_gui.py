from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

from PySide6.QtCore import Qt, QObject, QThread, QUrl, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .imu import load_imu_csv, load_slamimu, summarize_rate
from .models import VideoInput
from .pipeline import StabilizationJob, normalize_image_algorithm, run_job
from .process import hidden_subprocess_kwargs
from .video_probe import classify_layout, ffprobe_json, format_duration_s, parse_rate, primary_video_stream, stream_duration_s


ACTIVE_IMU_ALGORITHM = "gyro-acc-fusion"
ACTIVE_IMU_ALGORITHM_LABEL = "6D VQF 陀螺 + 加速度融合"
ACTIVE_IMAGE_ALGORITHM = "reference-renderer"
ACTIVE_IMAGE_ALGORITHM_LABEL = "Reference Renderer 标定鱼眼重投影"
ACTIVE_STABILIZATION_MODE = "horizon-lock"
ACTIVE_STABILIZATION_MODE_LABEL = "地平线防抖模式"


def resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2]))
    return base / relative


def app_path(relative: str) -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / relative
    return Path(relative).resolve()


class Worker(QObject):
    finished = Signal(str)
    failed = Signal(str)
    progress = Signal(int, str, float, float)

    def __init__(self, action: str, payload: dict[str, str]) -> None:
        super().__init__()
        self.action = action
        self.payload = payload
        self.started_at = 0.0

    def run(self) -> None:
        self.started_at = time.monotonic()
        try:
            if self.action == "inspect":
                self.finished.emit(self._inspect())
            else:
                self.finished.emit(self._prototype())
        except Exception:
            self.failed.emit(traceback.format_exc())

    def _inspect(self) -> str:
        self._emit_progress(5, "Validating selected files")
        mode = self.payload.get("mode", "sbs")
        video_path = Path(self.payload["sbs"] if mode == "sbs" else self.payload["left"])
        imu_path = Path(self.payload["imu"])
        if not str(video_path):
            raise ValueError("Please select an SBS video, or switch to separate left/right mode and select a left video.")
        if not str(imu_path):
            raise ValueError("Please select an IMU or SLAM motion file.")
        self._emit_progress(20, "Reading video metadata")
        probe = ffprobe_json(video_path)
        stream = primary_video_stream(probe)
        self._emit_progress(45, "Parsing IMU file")
        motion = load_slamimu(imu_path) if imu_path.suffix.lower() == ".slamimu" else None
        samples = motion.samples if motion else load_imu_csv(imu_path)
        width = int(stream.get("width", 0))
        height = int(stream.get("height", 0))
        duration = stream_duration_s(stream) or format_duration_s(probe)
        fps = parse_rate(stream.get("avg_frame_rate")) or parse_rate(stream.get("r_frame_rate"))
        frames = int(stream.get("nb_frames", 0) or 0)
        imu_duration = samples[-1].timestamp_s - samples[0].timestamp_s
        data_streams = [s for s in probe.get("streams", []) if s.get("codec_type") == "data"]

        report = {
            "video": {
                "path": str(video_path),
                "codec": stream.get("codec_name"),
                "width": width,
                "height": height,
                "layout": classify_layout(width, height),
                "duration_s": duration,
                "frame_rate": fps,
                "frames": frames,
                "data_stream_count": len(data_streams),
            },
            "imu": {
                "path": str(imu_path),
                "format": "slamimu_sqlite" if motion else "legacy_table",
                "samples": len(samples),
                "duration_s": imu_duration,
                "rate_hz": summarize_rate(samples),
                "timed_video_frames": len(motion.frame_times_s) if motion else None,
                "rolling_shutter_skew_s": motion.rolling_shutter_skew_s if motion else None,
            },
            "sync": {
                "duration_delta_s": imu_duration - duration,
                "imu_samples_per_video_frame": len(samples) / frames if frames else 0,
            },
        }

        report_path = Path(self.payload["report"])
        report_path.parent.mkdir(parents=True, exist_ok=True)
        self._emit_progress(85, "Writing analysis report")
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        self._emit_progress(100, "Analysis complete")

        return (
            "Pair analysis complete.\n"
            f"Video: {width}x{height}, {stream.get('codec_name')}, {duration:.6f}s, {fps:.3f}fps, {frames} frames\n"
            f"Layout: {report['video']['layout']}\n"
            f"IMU: {len(samples)} samples, {imu_duration:.6f}s, {report['imu']['rate_hz']:.3f}Hz"
            f"{f', {len(motion.frame_times_s)} timed frames' if motion else ''}\n"
            f"IMU - video duration: {report['sync']['duration_delta_s']:.6f}s\n"
            f"Embedded data streams: {len(data_streams)}\n"
            f"Report: {report_path}\n"
        )

    def _prototype(self) -> str:
        self._emit_progress(1, "Starting stabilization prototype")
        mode = self.payload["mode"]
        calibration = self.payload.get("calibration", "")
        output = Path(self.payload["output"])
        video = VideoInput(
            mode=mode,
            input_sbs=Path(self.payload["sbs"]) if mode == "sbs" else None,
            input_left=Path(self.payload["left"]) if mode == "dual" else None,
            input_right=Path(self.payload["right"]) if mode == "dual" else None,
        )
        plan = run_job(
            StabilizationJob(
                video=video,
                imu_path=Path(self.payload["imu"]),
                lens_profile_path=Path(self.payload["lens"]),
                calibration_path=Path(calibration) if calibration else None,
                output_path=output,
                imu_offset_s=0.0,
                gyro_scale=1.0,
                smooth_ms=float(self.payload.get("smooth_ms", "1000") or 1000),
                max_correction_deg=float(self.payload.get("max_correction_deg", "15") or 15),
                imu_algorithm=ACTIVE_IMU_ALGORITHM,
                stabilization_mode=ACTIVE_STABILIZATION_MODE,
                image_algorithm=ACTIVE_IMAGE_ALGORITHM,
                distortion_correction=self.payload.get("distortion_correction", "true") == "true",
                field_of_view_deg=float(self.payload.get("field_of_view_deg", "180") or 180),
                output_projection=self.payload.get("output_projection", "VR180 fisheye SBS"),
                metadata_target=self.payload.get("metadata_target", "YouTube VR180"),
                render_width=int(self.payload.get("render_width", "1920") or 1920),
                render_backend_preference=self.payload.get("render_backend", "auto"),
            ),
            progress=self._emit_progress,
        )
        plan_data = json.loads(plan.read_text(encoding="utf-8"))
        renderer = plan_data.get("export", {}).get("renderer_execution", {})
        renderer_name = renderer.get("name", "Unknown renderer")
        renderer_api = renderer.get("api", "CPU")
        fallback_reason = renderer.get("fallback_reason")
        fallback_line = f"GPU fallback reason: {fallback_reason}\n" if fallback_reason else ""
        return (
            "Stabilization run complete.\n"
            "Reference Renderer active: calibrated fisheye reprojection rendered stabilized pixels.\n"
            f"Actual render device: {renderer_name} ({renderer_api})\n"
            f"{fallback_line}"
            f"Output video: {output}\n"
            f"Plan JSON: {plan}\n"
            f"Per-frame diagnostics: {output.with_suffix(output.suffix + '.frames.csv')}\n"
        )

    def _emit_progress(self, percent: int, message: str) -> None:
        if self.started_at <= 0.0:
            self.started_at = time.monotonic()
        elapsed = max(0.0, time.monotonic() - self.started_at)
        if percent <= 0:
            remaining = 0.0
        elif percent >= 100:
            remaining = 0.0
        else:
            remaining = elapsed * (100.0 - percent) / percent
        self.progress.emit(percent, message, elapsed, remaining)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SLAM XCAM Spatial Video Stabilizer")
        self.resize(1280, 780)
        self.setMinimumSize(760, 520)
        self.thread: QThread | None = None
        self.worker: Worker | None = None
        self.left_tab_buttons: list[QPushButton] = []
        self.left_stack: QStackedWidget | None = None

        self.sbs = QLineEdit()
        self.left = QLineEdit()
        self.right = QLineEdit()
        self.imu = QLineEdit()
        self.calibration = QLineEdit()
        self.output_dir = QLineEdit()
        self.output = QLineEdit()
        self.report = QLineEdit()
        self.imu_offset = QDoubleSpinBox()
        self.imu_offset.setRange(-10.0, 10.0)
        self.imu_offset.setDecimals(3)
        self.imu_offset.setSingleStep(0.005)
        self.imu_offset.setValue(-0.167)
        self.imu_offset.setSuffix(" s")
        self.gyro_scale = QDoubleSpinBox()
        self.gyro_scale.setRange(0.10, 3.00)
        self.gyro_scale.setDecimals(3)
        self.gyro_scale.setSingleStep(0.01)
        self.gyro_scale.setValue(0.45)
        self.gyro_scale.setSuffix(" x")
        self.smooth_ms = QDoubleSpinBox()
        self.smooth_ms.setRange(0.0, 5000.0)
        self.smooth_ms.setValue(1000.0)
        self.smooth_ms.setSingleStep(100.0)
        self.smooth_ms.setSuffix(" ms")
        self.max_correction = QDoubleSpinBox()
        self.max_correction.setRange(1.0, 90.0)
        self.max_correction.setValue(15.0)
        self.max_correction.setSingleStep(1.0)
        self.max_correction.setSuffix(" deg")
        self.output_projection = QComboBox()
        self.output_projection.addItems(["VR180 fisheye SBS", "VR180 half-equirect SBS (planned)"])
        self.metadata_target = QComboBox()
        self.metadata_target.addItems(["YouTube VR180", "Apple Vision Pro APMP (planned)", "Both (planned)"])
        self.render_width = QSpinBox()
        self.render_width.setRange(640, 7680)
        self.render_width.setSingleStep(320)
        self.render_width.setValue(1920)
        self.render_width.setSuffix(" px")
        self.render_backend = QComboBox()
        self.render_backend.addItem("自动选择: 独显 GPU > 集显 > CPU", "auto")
        self.render_backend.addItem("优先独显 GPU", "discrete_gpu")
        self.render_backend.addItem("优先集显", "integrated_gpu")
        self.render_backend.addItem("仅 CPU", "cpu")
        self.video_duration_text = "--:--"
        self.video_size_text = "--"

        self.model = QComboBox()
        self.model.addItem("2025", str(resource_path("config/lenses/slam_xcam_2025.json")))
        self.model.addItem("2026", str(resource_path("config/lenses/slam_xcam_2026.json")))
        self.model.setCurrentText("2026")

        self.video_mode = QComboBox()
        self.video_mode.addItem("鱼眼", "sbs_fisheye")
        self.video_mode.addItem("等轴距投影", "sbs_equirect")

        self.distortion_correction = QComboBox()
        self.distortion_correction.addItem("是", True)
        self.distortion_correction.addItem("否", False)

        self.field_of_view = QComboBox()
        self.field_of_view.addItem("180°", 180)
        self.field_of_view.addItem("190°", 190)
        self.field_of_view.addItem("200°", 200)

        self.lut = QComboBox()
        self.lut.addItem("None", "none")
        self.lut.addItem("SLAM Natural", "slam-natural")
        self.lut.addItem("VR180 Soft Contrast", "vr180-soft-contrast")
        self.lut.addItem("Indoor Warm", "indoor-warm")
        self.lut.addItem("Outdoor Clear", "outdoor-clear")
        self.lut.addItem("Skin Tone Protect", "skin-tone-protect")
        self.lut.addItem("Flat Log Preview", "flat-log-preview")
        self.exposure = self._slider(-20, 20, 0)
        self.contrast = self._slider(-50, 50, 0)
        self.saturation = self._slider(-50, 50, 0)
        self.temperature = self._slider(-100, 100, 0)

        self.imu_algorithm_label = QLabel(ACTIVE_IMU_ALGORITHM_LABEL)
        self.imu_algorithm_label.setObjectName("fixedAlgorithm")
        self.imu_algorithm_info = QLabel()
        self.imu_algorithm_info.setWordWrap(True)

        self.image_algorithm_label = QLabel(ACTIVE_IMAGE_ALGORITHM_LABEL)
        self.image_algorithm_label.setObjectName("fixedAlgorithm")
        self.image_algorithm_info = QLabel()
        self.image_algorithm_info.setWordWrap(True)

        self.stabilization_mode_label = QLabel(ACTIVE_STABILIZATION_MODE_LABEL)
        self.stabilization_mode_label.setObjectName("fixedAlgorithm")
        self.stabilization_mode_info = QLabel()
        self.stabilization_mode_info.setWordWrap(True)

        for field in [
            self.sbs,
            self.imu,
            self.model,
            self.video_mode,
            self.distortion_correction,
            self.field_of_view,
            self.lut,
            self.imu_algorithm_label,
            self.image_algorithm_label,
            self.stabilization_mode_label,
            self.render_backend,
        ]:
            field.setFixedHeight(40)

        self.calibration_presets = QComboBox()
        self.calibration_presets.addItem("Official Calibration Runtime (auto)", "")
        self.calibration_presets.currentIndexChanged.connect(self._apply_calibration_preset)
        self.model.currentIndexChanged.connect(self._apply_model_defaults)
        self.model.currentIndexChanged.connect(self._refresh_preview_details)
        self.video_mode.currentIndexChanged.connect(self._refresh_preview_details)
        self.distortion_correction.currentIndexChanged.connect(self._refresh_preview_details)
        self.field_of_view.currentIndexChanged.connect(self._refresh_preview_details)
        self.lut.currentIndexChanged.connect(self._refresh_preview_details)
        self.render_backend.currentIndexChanged.connect(self._refresh_preview_details)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.preview_title = QLabel("No analysis yet")
        self.preview_stats = QLabel()
        self.auto_match_label = QLabel()
        self.auto_match_label.setWordWrap(True)
        self.preview_image = QLabel()
        self.preview_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_image.setStyleSheet("background: #05070a; color: #a9b7c6;")
        self.video_widget = QVideoWidget()
        self.player = QMediaPlayer(self)
        self.audio = QAudioOutput(self)
        self.audio.setMuted(True)
        self.player.setAudioOutput(self.audio)
        self.player.setVideoOutput(self.video_widget)
        self.player.playbackStateChanged.connect(self._sync_playback_button)
        self.playback_button: QPushButton | None = None
        self.preview_timeline = QProgressBar()
        self.preview_timeline.setRange(0, 100)
        self.preview_timeline.setValue(0)
        self.preview_timeline.setTextVisible(False)
        self.analyze_button: QPushButton | None = None
        self.start_buttons: list[QPushButton] = []

        self._build()
        self._apply_model_defaults()
        self._refresh_algorithm_info()
        self._refresh_default_outputs()
        if self.sbs.text():
            self._auto_match_imu(Path(self.sbs.text()), force=False)
            self._set_preview_video(Path(self.sbs.text()), load_thumbnail=False)
        self._refresh_preview_details()

    def _build(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        header = QHBoxLayout()
        logo = QFrame()
        logo.setFixedSize(34, 34)
        logo.setObjectName("logo")
        title = QLabel("SLAM XCAM Spatial Video Stabilizer")
        title.setStyleSheet("font-size: 20px; font-weight: 700;")
        header.addWidget(logo)
        header.addWidget(title)
        header.addStretch(1)
        root.addLayout(header)

        workspace = QSplitter()
        workspace.setChildrenCollapsible(False)
        workspace.addWidget(self._media_panel())
        workspace.addWidget(self._preview_panel())
        workspace.setSizes([360, 720])
        root.addWidget(workspace, 1)

        bottom = QGroupBox("Job Log")
        bottom_layout = QVBoxLayout(bottom)
        self.log.setMinimumHeight(90)
        self.log.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        bottom_layout.addWidget(self.log)
        root.addWidget(bottom)
        self.setCentralWidget(central)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f5f6f8; color: #111827; font-family: "Segoe UI", "Microsoft YaHei", Arial, sans-serif; }
            QFrame#logo { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0f172a, stop:1 #1f6feb); border-radius: 8px; }
            QGroupBox { font-weight: 600; border: 1px solid #d9dde3; border-radius: 8px; margin-top: 10px; padding: 10px; background: #ffffff; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { background: #1f6feb; color: white; border: none; border-radius: 6px; padding: 8px 12px; font-weight: 600; }
            QPushButton:hover { background: #2b7df5; }
            QLineEdit, QComboBox { background: #ffffff; border: 1px solid #cfd6df; border-radius: 6px; padding-left: 10px; padding-right: 10px; }
            QLabel#fixedAlgorithm { background: #f8fafc; border: 1px solid #cfd6df; border-radius: 6px; padding: 0 10px; color: #111827; font-weight: 600; }
            QDoubleSpinBox, QPlainTextEdit { background: #ffffff; border: 1px solid #cfd6df; border-radius: 5px; padding: 5px; }
            QSpinBox { background: #ffffff; border: 1px solid #cfd6df; border-radius: 5px; padding: 5px; }
            QPlainTextEdit { font-family: Consolas, monospace; }
            QFrame#mediaPanel { background: #ffffff; border: 1px solid #d9dde3; border-radius: 8px; }
            QFrame#segmentedTabs { background: #eef2f7; border: 1px solid #d7dde7; border-radius: 8px; }
            QPushButton[leftTab="true"] { background: transparent; color: #4b5563; border: none; border-radius: 6px; padding: 8px 8px; }
            QPushButton[leftTab="true"][active="true"] { background: #ffffff; color: #1f6feb; }
            """
        )

    def _media_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("mediaPanel")
        panel.setFixedWidth(380)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        tab_bar = QFrame()
        tab_bar.setObjectName("segmentedTabs")
        tab_layout = QHBoxLayout(tab_bar)
        tab_layout.setContentsMargins(4, 4, 4, 4)
        tab_layout.setSpacing(8)

        self.left_stack = QStackedWidget()
        for index, (label, widget) in enumerate(
            [
                ("Media Setting", self._media_setting_tab()),
                ("Color Correction", self._color_correction_tab()),
                ("算法策略", self._algorithm_strategy_tab()),
            ]
        ):
            button = QPushButton(label)
            button.setProperty("leftTab", True)
            button.setProperty("active", index == 0)
            button.clicked.connect(lambda checked=False, i=index: self._switch_left_tab(i))
            self.left_tab_buttons.append(button)
            tab_layout.addWidget(button, 1)
            self.left_stack.addWidget(widget)

        layout.addWidget(tab_bar)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(self.left_stack)
        layout.addWidget(scroll, 1)

        start = QPushButton("Start Stabilization")
        start.clicked.connect(self.run_prototype)
        self.start_buttons.append(start)
        start.setMinimumHeight(46)
        layout.addWidget(start)
        return panel

    def _switch_left_tab(self, index: int) -> None:
        if self.left_stack is not None:
            self.left_stack.setCurrentIndex(index)
        for button_index, button in enumerate(self.left_tab_buttons):
            button.setProperty("active", button_index == index)
            button.style().unpolish(button)
            button.style().polish(button)

    def _media_setting_tab(self) -> QWidget:
        tab = QWidget()
        tab.setStyleSheet("background: #ffffff;")
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._field("SLAM XCAM Model", self.model))
        layout.addWidget(self._field("Video mode", self.video_mode))
        layout.addWidget(self._field("畸变矫正", self.distortion_correction))
        layout.addWidget(self._field("视场角", self.field_of_view))
        layout.addWidget(self._field("Calibration source", self.calibration_presets))
        layout.addWidget(
            self._field(
                "Custom calibration JSON",
                self._file_row(
                    self.calibration,
                    "Calibration JSON (*.json);;All files (*.*)",
                    "Choose JSON",
                ),
            )
        )
        layout.addWidget(
            self._field(
                "Choose video",
                self._file_row(self.sbs, "Video files (*.mp4 *.mov *.mkv);;All files (*.*)", "Choose video"),
            )
        )
        hint = QLabel("选择视频后优先匹配同目录的 video_motion.slamimu，也兼容旧版 video_imu.csv。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #6b7280; font-size: 12px;")
        layout.addWidget(hint)
        layout.addWidget(
            self._field(
                "Choose IMU",
                self._file_row(
                    self.imu,
                    "SLAM motion files (*.slamimu);;Legacy IMU files (*.csv *.xlsx *.xlsm);;All files (*.*)",
                    "Choose IMU",
                ),
            )
        )
        self.auto_match_label.setStyleSheet(
            "background: #e8f1ff; color: #174ea6; border: 1px solid #c7dcff; border-radius: 8px; padding: 8px;"
        )
        self.auto_match_label.hide()
        layout.addWidget(self.auto_match_label)
        layout.addStretch(1)
        return tab

    def _color_correction_tab(self) -> QWidget:
        tab = QWidget()
        tab.setStyleSheet("background: #ffffff;")
        form = QVBoxLayout(tab)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(8)
        form.addWidget(self._field("LUT", self.lut))
        form.addWidget(self._field("Exposure", self.exposure))
        form.addWidget(self._field("Contrast", self.contrast))
        form.addWidget(self._field("Saturation", self.saturation))
        form.addWidget(self._field("White balance", self.temperature))
        note = QLabel("LUT library: config/luts/library.json. LUT files will be added under config/luts/files/.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #6b7280; font-size: 12px;")
        form.addWidget(note)
        form.addStretch(1)
        return tab

    def _algorithm_strategy_tab(self) -> QWidget:
        tab = QWidget()
        tab.setStyleSheet("background: #ffffff;")
        form = QVBoxLayout(tab)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(8)
        self.imu_algorithm_info.setStyleSheet("background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 10px; color: #4b5563;")
        self.image_algorithm_info.setStyleSheet("background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 10px; color: #4b5563;")
        self.stabilization_mode_info.setStyleSheet("background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 10px; color: #4b5563;")
        form.addWidget(self._field("IMU处理算法", self.imu_algorithm_label))
        form.addWidget(self.imu_algorithm_info)
        form.addWidget(self._field("图像处理算法", self.image_algorithm_label))
        form.addWidget(self.image_algorithm_info)
        form.addWidget(self._field("防抖模式", self.stabilization_mode_label))
        form.addWidget(self.stabilization_mode_info)
        form.addWidget(self._field("渲染设备", self.render_backend))
        form.addStretch(1)
        return tab

    def _section_title(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setStyleSheet("font-size: 18px; font-weight: 700; color: #111827; margin-top: 2px;")
        return label

    def _field(self, label_text: str, control: QWidget) -> QWidget:
        field = QWidget()
        field.setMinimumHeight(64)
        layout = QVBoxLayout(field)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        label = QLabel(label_text)
        label.setStyleSheet("color: #374151; font-size: 13px; font-weight: 600;")
        layout.addWidget(label)
        layout.addWidget(control)
        return field

    def _preview_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("previewPanel")
        panel.setMinimumWidth(360)
        panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        panel.setStyleSheet("QFrame#previewPanel { background: #ffffff; border: 1px solid #d9dde3; border-radius: 8px; }")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        preview_head = QWidget()
        preview_head.setStyleSheet("background: #ffffff; border-bottom: 1px solid #d9dde3;")
        head_layout = QVBoxLayout(preview_head)
        head_layout.setContentsMargins(18, 14, 18, 14)
        head_layout.setSpacing(6)
        title_row = QHBoxLayout()
        title_row.setContentsMargins(0, 0, 0, 0)
        title_row.setSpacing(12)
        self.preview_title.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.preview_title.setStyleSheet("font-size: 18px; font-weight: 700; color: #111827;")
        self.preview_title.setMinimumWidth(0)
        self.preview_title.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.playback_button = QPushButton("Play")
        self.playback_button.setFixedWidth(90)
        self.playback_button.setMinimumHeight(36)
        self.playback_button.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.playback_button.setStyleSheet(
            "background: #ffffff; color: #111827; border: 1px solid #cfd6df; border-radius: 6px; padding: 8px 10px; font-weight: 600;"
        )
        self.playback_button.clicked.connect(self.toggle_playback)
        self.preview_title.setMaximumWidth(320)
        title_row.addWidget(self.preview_title)
        title_row.addStretch(1)
        title_row.addWidget(self.playback_button, 0, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight)
        self.preview_stats.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.preview_stats.setWordWrap(True)
        self.preview_stats.setStyleSheet("color: #4b5563; font-size: 13px;")
        self.preview_stats.setMinimumWidth(0)
        self.preview_stats.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        head_layout.addLayout(title_row)
        head_layout.addWidget(self.preview_stats)
        layout.addWidget(preview_head)

        preview = QFrame()
        preview.setMinimumWidth(0)
        preview.setMinimumHeight(220)
        preview.setStyleSheet("QFrame { background: #0d1117; border: none; } QLabel { color: #dbe7f3; }")
        preview_layout = QVBoxLayout(preview)
        preview_layout.setContentsMargins(24, 24, 24, 24)
        self.video_widget.setMinimumHeight(180)
        self.video_widget.setMinimumWidth(0)
        self.video_widget.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.video_widget.setStyleSheet("background: #05070a;")
        self.video_widget.hide()
        self.preview_image.setMinimumHeight(180)
        self.preview_image.setMinimumWidth(0)
        self.preview_image.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.preview_image.setStyleSheet("background: #05070a; color: #a9b7c6; border: 1px solid #263241; border-radius: 8px;")
        preview_layout.addWidget(self.preview_image, 1)
        preview_layout.addWidget(self.video_widget, 1)
        layout.addWidget(preview, 1)

        job = QWidget()
        job_layout = QVBoxLayout(job)
        job_layout.setContentsMargins(18, 14, 18, 18)
        progress_row = QHBoxLayout()
        self.progress_text = QLabel("Ready")
        self.progress_percent = QLabel("0%")
        progress_row.addWidget(self.progress_text)
        progress_row.addStretch(1)
        progress_row.addWidget(self.progress_percent)
        job_layout.addLayout(progress_row)
        job_layout.addWidget(self.preview_timeline)
        self.time_meta = QLabel("Elapsed 00:00 · ETA --:--")
        self.time_meta.setStyleSheet("color: #6b7280; font-size: 13px;")
        job_layout.addWidget(self.time_meta)
        layout.addWidget(job)
        return panel

    def _slider(self, minimum: int, maximum: int, value: int) -> QSlider:
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(value)
        slider.setTickPosition(QSlider.TickPosition.NoTicks)
        return slider

    def _file_row(self, line: QLineEdit, filter_text: str, button_text: str = "Browse") -> QWidget:
        row = QWidget()
        row.setMinimumWidth(0)
        row.setFixedHeight(44)
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        line.setMinimumWidth(0)
        line.setFixedHeight(44)
        button = QPushButton(button_text)
        button.setFixedWidth(138)
        button.setFixedHeight(44)
        button.setStyleSheet(
            "background: #1f6feb; color: #ffffff; border: none; border-radius: 6px; padding: 8px 10px; font-weight: 600;"
        )
        button.clicked.connect(lambda: self._pick_file(line, filter_text))
        layout.addWidget(line, 1)
        layout.addWidget(button)
        return row

    def _folder_row(self, line: QLineEdit) -> QWidget:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        button = QPushButton("Browse")
        button.clicked.connect(lambda: self._pick_folder(line))
        layout.addWidget(line, 1)
        layout.addWidget(button)
        return row

    def _save_row(self, line: QLineEdit, filter_text: str) -> QWidget:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        button = QPushButton("Save As")
        button.clicked.connect(lambda: self._pick_save(line, filter_text))
        layout.addWidget(line, 1)
        layout.addWidget(button)
        return row

    def _pick_file(self, line: QLineEdit, filter_text: str) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select file", "", filter_text)
        if path:
            line.setText(path)
            if line is self.imu:
                self._apply_motion_file_defaults(Path(path))
            if line is self.sbs or line is self.left:
                video_path = Path(path)
                self._set_preview_video(video_path, load_thumbnail=False)
                self._auto_match_imu(video_path, force=True)
                self._refresh_default_outputs(force=True)
                self._refresh_video_probe_summary(video_path)

    def _pick_save(self, line: QLineEdit, filter_text: str) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Select output", line.text(), filter_text)
        if path:
            line.setText(path)

    def _pick_folder(self, line: QLineEdit) -> None:
        path = QFileDialog.getExistingDirectory(self, "Select save folder", line.text())
        if path:
            line.setText(path)
            self._refresh_default_outputs(force=True)

    def _apply_calibration_preset(self) -> None:
        path = self.calibration_presets.currentData()
        self.calibration.setText(str(path or ""))

    def _apply_model_defaults(self) -> None:
        if not hasattr(self, "calibration_presets"):
            return
        model = self.model.currentText()
        self.calibration.setText("")
        for index in range(self.calibration_presets.count()):
            if self.calibration_presets.itemData(index) == "":
                self.calibration_presets.setCurrentIndex(index)
                break
        self._refresh_preview_details()

    def _refresh_algorithm_info(self) -> None:
        self.imu_algorithm_info.setText(
            "官方 PyVQF 对 200Hz 陀螺和加速度做无磁姿态融合、零偏估计和静止检测。"
            "当前 pipeline 固定使用此算法。"
        )
        self.image_algorithm_info.setText(
            "输出像素转换为 VR180 ray，应用同一套双目 IMU 防抖旋转，"
            "再通过真实镜头 K/D 参数投影回源鱼眼。优先使用 OpenGL GPU，"
            "GPU 不可用时自动回退 CPU。"
        )
        self.stabilization_mode_info.setText(
            "当前只开发地平线防抖：平滑相机姿态并锁定横滚。"
            "普通防抖和其他模式暂不进入正式 pipeline。"
        )

    def _auto_match_imu(self, video: Path, force: bool = False) -> None:
        if not video:
            return
        if self.imu.text() and not force:
            return
        candidates = [
            video.with_name(f"{video.stem}_motion.slamimu"),
            video.with_name(f"{video.stem}_imu.csv"),
            video.with_name(f"{video.stem}_IMU.csv"),
            video.with_suffix(".csv"),
        ]
        for candidate in candidates:
            if candidate.exists():
                self.imu.setText(str(candidate))
                self._apply_motion_file_defaults(candidate)
                self.log.appendPlainText(f"Auto matched IMU: {candidate}\n")
                self.auto_match_label.setText(f"Auto matched IMU:\n{candidate.name}")
                self.auto_match_label.show()
                return
        if force:
            self.log.appendPlainText(f"No matching SLAM motion or legacy IMU file found next to video: {video.name}\n")
            self.auto_match_label.setText("No matching motion data found next to the selected video.")
            self.auto_match_label.show()

    def _apply_motion_file_defaults(self, path: Path) -> None:
        if path.suffix.lower() != ".slamimu":
            return
        self.imu_offset.setValue(0.0)
        self.gyro_scale.setValue(1.0)
        self.log.appendPlainText(
            "SLAM motion timing detected: fixed 6D VQF pipeline, gyro scale 1.0x, "
            "manual IMU offset set to 0s, and per-row rolling shutter enabled.\n"
        )

    def _selected_video_path(self) -> Path | None:
        text = self.sbs.text() if self._pipeline_video_mode() == "sbs" else self.left.text()
        return Path(text) if text else None

    def _pipeline_video_mode(self) -> str:
        mode = self.video_mode.currentData() or "sbs_fisheye"
        if str(mode).startswith("sbs_"):
            return "sbs"
        return str(mode)

    def _refresh_default_outputs(self, force: bool = False) -> None:
        video = self._selected_video_path()
        if not video:
            return
        save_dir = Path(self.output_dir.text()) if self.output_dir.text() else video.parent
        if not self.output_dir.text() or force:
            self.output_dir.setText(str(save_dir))
        default_output = save_dir / f"{video.stem}_stabilized_prototype.mp4"
        default_report = save_dir / f"{video.stem}_pair_report.json"
        if force or not self.output.text():
            self.output.setText(str(default_output))
        if force or not self.report.text():
            self.report.setText(str(default_report))
        self._refresh_preview_details()

    def _refresh_video_probe_summary(self, video: Path) -> None:
        try:
            probe = ffprobe_json(video)
            stream = primary_video_stream(probe)
            width = int(stream.get("width", 0) or 0)
            height = int(stream.get("height", 0) or 0)
            duration = stream_duration_s(stream) or format_duration_s(probe)
            self.video_size_text = f"{width} x {height}" if width and height else "--"
            self.video_duration_text = self._format_duration_label(duration)
        except Exception:
            self.video_size_text = "--"
            self.video_duration_text = "--:--"
        self._refresh_preview_details()

    def _format_duration_label(self, seconds: float) -> str:
        seconds = max(0.0, float(seconds or 0.0))
        minutes = int(seconds // 60)
        secs = seconds - minutes * 60
        return f"{minutes:02d}:{secs:05.2f}"

    def _refresh_preview_details(self) -> None:
        video = self._selected_video_path()
        video_folder = str(video.parent) if video else "--"
        save_folder = self.output_dir.text() or video_folder
        algorithm = (
            f"{ACTIVE_IMU_ALGORITHM_LABEL} + {ACTIVE_IMAGE_ALGORITHM_LABEL} + "
            f"{ACTIVE_STABILIZATION_MODE_LABEL}"
        )
        self.preview_stats.setText(
            f"SLAM XCAM Model {self.model.currentText()} · {self.video_mode.currentText()} · "
            f"畸变矫正 {self.distortion_correction.currentText()} · 视场角 {self.field_of_view.currentText()} · "
            f"{algorithm} · {self.render_backend.currentText()} · {self.video_duration_text} · {self.video_size_text}\n"
            f"Video folder: {video_folder}\n"
            f"Save folder: {save_folder}"
        )

    def _set_preview_video(self, path: Path, load_thumbnail: bool = False) -> None:
        if not path.exists():
            return
        self.preview_title.setText(path.name)
        self._refresh_video_probe_summary(path)
        if load_thumbnail:
            self._load_preview_thumbnail(path)
        else:
            self.preview_image.setText("Video selected. Preview thumbnail is skipped during analysis to keep the UI responsive.")
            self.preview_image.show()
            self.video_widget.hide()
        self.player.setSource(QUrl.fromLocalFile(str(path)))
        self.player.pause()

    def _load_preview_thumbnail(self, path: Path) -> None:
        thumb_dir = app_path("outputs")
        thumb_dir.mkdir(parents=True, exist_ok=True)
        thumb = thumb_dir / "_preview_frame.jpg"
        try:
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-ss",
                    "0.2",
                    "-i",
                    str(path),
                    "-frames:v",
                    "1",
                    "-vf",
                    "scale=960:-1",
                    str(thumb),
                ],
                check=True,
                capture_output=True,
                text=True,
                **hidden_subprocess_kwargs(),
            )
            pixmap = QPixmap(str(thumb))
            if pixmap.isNull():
                raise ValueError("preview thumbnail was empty")
            self.preview_image.setPixmap(
                pixmap.scaled(
                    self.preview_image.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )
            self.preview_image.show()
            self.video_widget.hide()
        except Exception as exc:
            self.preview_image.setText(f"Preview thumbnail failed: {exc}")
            self.preview_image.show()
            self.video_widget.hide()

    def toggle_playback(self) -> None:
        if self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()
        else:
            self.preview_image.hide()
            self.video_widget.show()
            self.player.play()

    def _sync_playback_button(self, state: QMediaPlayer.PlaybackState) -> None:
        if self.playback_button is None:
            return
        is_playing = state == QMediaPlayer.PlaybackState.PlayingState
        self.playback_button.setText("Pause" if is_playing else "Play")

    def _payload(self) -> dict[str, str]:
        self._refresh_default_outputs()
        return {
            "mode": self._pipeline_video_mode(),
            "video_mode_label": self.video_mode.currentText(),
            "distortion_correction": "true" if self.distortion_correction.currentData() else "false",
            "field_of_view_deg": str(self.field_of_view.currentData()),
            "sbs": self.sbs.text(),
            "left": self.left.text(),
            "right": self.right.text(),
            "imu": self.imu.text(),
            "lens": self.model.currentData(),
            "calibration": self.calibration.text(),
            "output": self.output.text(),
            "output_dir": self.output_dir.text(),
            "report": self.report.text(),
            "imu_offset_s": "0.000000",
            "gyro_scale": "1.000000",
            "smooth_ms": f"{self.smooth_ms.value():.3f}",
            "max_correction_deg": f"{self.max_correction.value():.3f}",
            "output_projection": self.output_projection.currentText(),
            "metadata_target": self.metadata_target.currentText(),
            "render_width": str(self.render_width.value()),
            "render_backend": self.render_backend.currentData(),
            "lut": self.lut.currentData(),
            "imu_algorithm": ACTIVE_IMU_ALGORITHM,
            "image_algorithm": ACTIVE_IMAGE_ALGORITHM,
            "stabilization_mode": ACTIVE_STABILIZATION_MODE,
        }

    def inspect_pair(self) -> None:
        try:
            self.log.appendPlainText("Analyze button clicked. Preparing video + IMU analysis...\n")
            video = self._selected_video_path()
            if video:
                self._set_preview_video(video, load_thumbnail=False)
                self._refresh_default_outputs()
            self._start("inspect", self._payload())
        except Exception:
            message = traceback.format_exc()
            self.log.appendPlainText(message)
            QMessageBox.critical(self, "Analyze failed before start", message)

    def run_prototype(self) -> None:
        self.log.appendPlainText("Start button clicked. Preparing stabilization job...\n")
        try:
            self._refresh_default_outputs()
            self._start("prototype", self._payload())
        except Exception:
            message = traceback.format_exc()
            self.log.appendPlainText(message)
            QMessageBox.critical(self, "Start failed before job launch", message)

    def _start(self, action: str, payload: dict[str, str]) -> None:
        if self.thread is not None:
            try:
                if self.thread.isRunning():
                    QMessageBox.warning(self, "Busy", "A job is already running.")
                    return
            except RuntimeError:
                self.thread = None
                self.worker = None
        try:
            self._validate_payload(action, payload)
        except Exception as exc:
            QMessageBox.warning(self, "Missing input", str(exc))
            self.log.appendPlainText(f"Cannot start {action}: {exc}\n")
            return
        if action == "prototype":
            self.log.appendPlainText(f"Output path: {payload.get('output', '')}\n")
            self.log.appendPlainText(f"Save folder: {payload.get('output_dir', '')}\n")
        self.log.appendPlainText(f"Starting {action}...\n")
        self.preview_timeline.setRange(0, 100)
        self.preview_timeline.setValue(0)
        self._set_busy(True)
        self.thread = QThread()
        self.worker = Worker(action, payload)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._progress)
        self.worker.finished.connect(self._done)
        self.worker.failed.connect(self._failed)
        self.worker.finished.connect(self.thread.quit)
        self.worker.failed.connect(self.thread.quit)
        self.thread.finished.connect(self._thread_finished)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

    def _validate_payload(self, action: str, payload: dict[str, str]) -> None:
        mode = payload.get("mode", "sbs")
        if mode == "sbs":
            if not payload.get("sbs"):
                raise ValueError("Please select an SBS video.")
        else:
            if not payload.get("left") or not payload.get("right"):
                raise ValueError("Please select both left and right videos.")
        if not payload.get("imu"):
            raise ValueError("Please select an IMU or SLAM motion file.")
        if action == "prototype" and not payload.get("output"):
            raise ValueError("Please choose an output MP4 path.")
        if action == "prototype" and normalize_image_algorithm(payload.get("image_algorithm", "")) != "reference-renderer":
            raise ValueError("目前已激活的是 Reference Renderer。STMap Renderer 已加入菜单，但还没有实现。")

    def _set_busy(self, busy: bool) -> None:
        if self.analyze_button:
            self.analyze_button.setEnabled(not busy)
        for button in self.start_buttons:
            button.setEnabled(not busy)
            button.setText("Running..." if busy else "Start Stabilization")

    def _thread_finished(self) -> None:
        self.thread = None
        self.worker = None

    def _format_seconds(self, seconds: float) -> str:
        seconds = max(0.0, seconds)
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes:02d}:{secs:02d}"

    def _progress(self, percent: int, message: str, elapsed: float, remaining: float) -> None:
        self.preview_timeline.setValue(max(0, min(100, percent)))
        self.preview_timeline.setFormat(f"{percent}%")
        if hasattr(self, "progress_text"):
            self.progress_text.setText(message)
        if hasattr(self, "progress_percent"):
            self.progress_percent.setText(f"{percent}%")
        if hasattr(self, "time_meta"):
            self.time_meta.setText(f"Elapsed {self._format_seconds(elapsed)} · ETA {self._format_seconds(remaining)}")
        self.log.appendPlainText(
            f"{percent:3d}% | elapsed {self._format_seconds(elapsed)} | "
            f"eta {self._format_seconds(remaining)} | {message}"
        )

    def _done(self, message: str) -> None:
        self.log.appendPlainText(message)
        self.log.appendPlainText("Finished.\n")
        self.preview_timeline.setRange(0, 100)
        self.preview_timeline.setValue(100)
        self.preview_timeline.setFormat("100%")
        if hasattr(self, "progress_text"):
            self.progress_text.setText("Complete")
        if hasattr(self, "progress_percent"):
            self.progress_percent.setText("100%")
        if hasattr(self, "time_meta"):
            self.time_meta.setText("Elapsed --:-- · ETA 00:00")
        if "Pair analysis complete." in message:
            lines = [line for line in message.splitlines() if line.strip()]
            self.preview_title.setText("Analysis complete")
            self.preview_stats.setText("\n".join(lines[1:5]))
            self.preview_timeline.setValue(100)
            QMessageBox.information(self, "Analysis complete", "Video + IMU analysis is complete.")
        elif "Stabilization run complete." in message or "Prototype run complete." in message:
            self.preview_title.setText("Stabilization complete")
            device_line = next(
                (line for line in message.splitlines() if line.startswith("Actual render device:")),
                "Actual render device: CPU",
            )
            self.preview_stats.setText(
                "Reference Renderer calibrated fisheye reprojection output was written.\n"
                f"{device_line}"
            )
            self.preview_timeline.setValue(100)
            QMessageBox.information(self, "Stabilization complete", "Prototype stabilization output is complete.")
        self._set_busy(False)

    def _failed(self, message: str) -> None:
        self.log.appendPlainText(message)
        self.log.appendPlainText("Failed.\n")
        self.preview_timeline.setRange(0, 100)
        self.preview_timeline.setValue(0)
        if hasattr(self, "progress_text"):
            self.progress_text.setText("Failed")
        if hasattr(self, "progress_percent"):
            self.progress_percent.setText("0%")
        self._set_busy(False)
        QMessageBox.critical(self, "Job failed", message)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
