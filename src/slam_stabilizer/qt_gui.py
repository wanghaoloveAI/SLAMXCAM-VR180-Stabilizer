from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from functools import lru_cache

from PySide6.QtCore import Qt, QObject, QSettings, QSize, QThread, QTimer, QUrl, Signal
from PySide6.QtGui import QAction, QActionGroup, QIcon, QPixmap
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMenu,
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
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .imu import load_imu_csv, load_slamimu, summarize_rate
from .models import VideoInput
from .pipeline import StabilizationJob, normalize_image_algorithm, run_job
from .process import find_tool, hidden_subprocess_kwargs
from .slam_source import (
    default_legacy_sbs_export_path,
    default_sbs_export_path,
    export_legacy_slam_source,
    export_slam_source,
    inspect_legacy_slam_source,
    inspect_slam_source,
    validate_processed_source,
)
from .video_probe import classify_layout, ffprobe_json, format_duration_s, parse_rate, primary_video_stream, stream_duration_s


ACTIVE_IMU_ALGORITHM = "gyro-acc-fusion"
ACTIVE_IMU_ALGORITHM_LABEL = "6D VQF 陀螺 + 加速度融合"
ACTIVE_IMAGE_ALGORITHM = "reference-renderer"
ACTIVE_IMAGE_ALGORITHM_LABEL = "Reference Renderer 标定鱼眼重投影"
ACTIVE_STABILIZATION_MODE = "horizon-lock"
ACTIVE_STABILIZATION_MODE_LABEL = "地平线防抖模式"
DEVICE_MEDIA_ROOT = "/sdcard/DCIM/VR180"
SUPPORTED_DEVICE_MEDIA_EXTENSIONS = {".jpg", ".jpeg", ".mp4", ".heif", ".heic", ".dng"}
LEGACY_DEVICE_COMPONENT_EXTENSIONS = {".m4a", ".json"}


UI_TEXT = {
    "menu": {"en": "Menu  ▾", "zh_CN": "菜单  ▾"},
    "ui_settings": {"en": "UI Settings", "zh_CN": "UI 设置"},
    "theme_minimal": {"en": "Minimal", "zh_CN": "极简风"},
    "theme_pixel": {"en": "Pixel", "zh_CN": "像素风"},
    "language": {"en": "Language", "zh_CN": "语言设置"},
    "english": {"en": "English", "zh_CN": "英语"},
    "chinese": {"en": "Simplified Chinese", "zh_CN": "简体中文"},
    "page_stabilization": {"en": "Stabilization", "zh_CN": "防抖处理"},
    "page_camera": {"en": "Camera Management", "zh_CN": "相机管理"},
    "page_editor": {"en": "Editing", "zh_CN": "剪辑"},
    "camera_description": {"en": "Connect SLAM XCAM and view device and storage information.", "zh_CN": "连接并识别 SLAM XCAM 的设备与存储信息。"},
    "refresh_device": {"en": "Refresh Device", "zh_CN": "刷新设备"},
    "device": {"en": "Device", "zh_CN": "设备"},
    "device_files": {"en": "Device Files", "zh_CN": "设备文件"},
    "detecting_device": {"en": "Detecting device...", "zh_CN": "正在检测设备…"},
    "detecting_usb": {"en": "Detecting USB device...", "zh_CN": "正在检测 USB 设备…"},
    "detecting_usb_xcam": {"en": "Detecting SLAM XCAM over USB...", "zh_CN": "正在通过 USB 检测 SLAM XCAM…"},
    "detecting": {"en": "Detecting...", "zh_CN": "正在检测…"},
    "connected_serial": {"en": "Connected · USB · Serial {serial}", "zh_CN": "已连接 · USB · 序列号 {serial}"},
    "device_button_connected": {"en": "SLAM XCAM\nUSB · Connected\n{serial}", "zh_CN": "SLAM XCAM\nUSB · 已连接\n{serial}"},
    "available_storage": {"en": "{available} available / {total}", "zh_CN": "{available} 可用 / {total}"},
    "not_detected": {"en": "Not Detected", "zh_CN": "未检测到"},
    "device_not_detected": {"en": "SLAM XCAM not detected\nCheck the USB connection", "zh_CN": "未检测到 SLAM XCAM\n请检查 USB 连接"},
    "storage": {"en": "Storage", "zh_CN": "存储空间"},
    "processor": {"en": "Processor", "zh_CN": "处理器型号"},
    "reload": {"en": "Reload", "zh_CN": "重新读取"},
    "export_selected": {"en": "Export Selected", "zh_CN": "导出所选文件"},
    "media_help": {"en": "Connect the camera to read media and legacy four-file stereo sessions.", "zh_CN": "连接设备后读取媒体文件和旧版四文件双目会话。"},
    "reading_media": {"en": "Reading /sdcard/DCIM/VR180...", "zh_CN": "正在读取 /sdcard/DCIM/VR180…"},
    "preparing_export": {"en": "Preparing export...", "zh_CN": "正在准备导出文件…"},
    "connect_first": {"en": "SLAM XCAM is not connected. Refresh the device first.", "zh_CN": "尚未连接 SLAM XCAM，请先刷新设备。"},
    "editor_title": {"en": "Editing (In Development)", "zh_CN": "剪辑功能（开发中）"},
    "in_development": {"en": "In Development", "zh_CN": "开发中"},
    "editor_description": {"en": "Media import, trimming, timeline editing, color tools, and VR180 metadata export will be added later.", "zh_CN": "后续将加入素材导入、基础剪切、时间线、色彩调整和 VR180 元数据导出。"},
    "editor_detail": {"en": "Editing is not available in this version. Stabilization and Camera Management remain fully independent.", "zh_CN": "当前版本暂不提供剪辑处理，防抖和相机管理页面可正常独立使用。"},
    "media_setting": {"en": "Media Setting", "zh_CN": "媒体设置"},
    "color_correction": {"en": "Color Correction", "zh_CN": "色彩校正"},
    "algorithm_strategy": {"en": "Algorithm Strategy", "zh_CN": "算法策略"},
    "camera_model": {"en": "SLAM XCAM Model", "zh_CN": "SLAM XCAM 型号"},
    "video_mode": {"en": "Video Mode", "zh_CN": "视频模式"},
    "choose_video": {"en": "Choose Video", "zh_CN": "选择视频"},
    "choose_imu": {"en": "Choose IMU", "zh_CN": "选择 IMU"},
    "imu_match_hint": {"en": "After selecting a video, Studio first looks for video_motion.slamimu in the same folder, with legacy video_imu.csv support.", "zh_CN": "选择视频后优先匹配同目录的 video_motion.slamimu，也兼容旧版 video_imu.csv。"},
    "exposure": {"en": "Exposure", "zh_CN": "曝光"},
    "contrast": {"en": "Contrast", "zh_CN": "对比度"},
    "saturation": {"en": "Saturation", "zh_CN": "饱和度"},
    "white_balance": {"en": "White Balance", "zh_CN": "白平衡"},
    "none": {"en": "None", "zh_CN": "无"},
    "lut_note": {"en": "LUT library: config/luts/library.json. LUT files will be added under config/luts/files/.", "zh_CN": "LUT 库：config/luts/library.json。后续 LUT 文件将放入 config/luts/files/。"},
    "distortion": {"en": "Distortion Correction", "zh_CN": "畸变矫正"},
    "field_of_view": {"en": "Field of View", "zh_CN": "视场角"},
    "fisheye": {"en": "Fisheye", "zh_CN": "鱼眼"},
    "equirect": {"en": "Equirectangular", "zh_CN": "等轴距投影"},
    "yes": {"en": "Yes", "zh_CN": "是"},
    "no": {"en": "No", "zh_CN": "否"},
    "imu_algorithm": {"en": "IMU Processing Algorithm", "zh_CN": "IMU处理算法"},
    "image_algorithm": {"en": "Image Processing Algorithm", "zh_CN": "图像处理算法"},
    "stabilization_mode": {"en": "Stabilization Mode", "zh_CN": "防抖模式"},
    "render_device": {"en": "Render Device", "zh_CN": "渲染设备"},
    "backend_auto": {"en": "Auto: discrete GPU > integrated GPU > CPU", "zh_CN": "自动选择: 独显 GPU > 集显 > CPU"},
    "backend_discrete": {"en": "Prefer discrete GPU", "zh_CN": "优先独显 GPU"},
    "backend_integrated": {"en": "Prefer integrated GPU", "zh_CN": "优先集显"},
    "backend_cpu": {"en": "CPU only", "zh_CN": "仅 CPU"},
    "pipeline_summary": {"en": "6D VQF · Reference Renderer · Horizon Lock", "zh_CN": "6D VQF · Reference Renderer · 地平线锁定"},
    "duration": {"en": "Duration", "zh_CN": "时长"},
    "resolution": {"en": "Resolution", "zh_CN": "分辨率"},
    "active_imu": {"en": "6D VQF Gyroscope + Accelerometer Fusion", "zh_CN": "6D VQF 陀螺 + 加速度融合"},
    "active_image": {"en": "Reference Renderer Calibrated Fisheye Reprojection", "zh_CN": "Reference Renderer 标定鱼眼重投影"},
    "active_horizon": {"en": "Horizon Lock", "zh_CN": "地平线防抖模式"},
    "imu_info": {"en": "Official PyVQF fuses 200 Hz gyroscope and accelerometer data without magnetometer input, including bias estimation and rest detection. This is the active pipeline algorithm.", "zh_CN": "官方 PyVQF 对 200Hz 陀螺和加速度做无磁姿态融合、零偏估计和静止检测。当前 pipeline 固定使用此算法。"},
    "image_info": {"en": "Output pixels are converted to VR180 rays, corrected with one shared stereo IMU rotation, then projected back into the source fisheye using measured K/D calibration. OpenGL GPU is preferred with automatic CPU fallback.", "zh_CN": "输出像素转换为 VR180 ray，应用同一套双目 IMU 防抖旋转，再通过真实镜头 K/D 参数投影回源鱼眼。优先使用 OpenGL GPU，GPU 不可用时自动回退 CPU。"},
    "horizon_info": {"en": "The active mode smooths camera orientation and locks roll to stabilize the horizon. Other stabilization modes are not part of the production pipeline yet.", "zh_CN": "当前只开发地平线防抖：平滑相机姿态并锁定横滚。普通防抖和其他模式暂不进入正式 pipeline。"},
    "video_folder": {"en": "Video folder", "zh_CN": "视频文件夹"},
    "save_folder": {"en": "Save folder", "zh_CN": "保存文件夹"},
    "job_log": {"en": "Job Log", "zh_CN": "任务日志"},
    "start_stabilization": {"en": "Start Stabilization", "zh_CN": "开始防抖"},
    "browse": {"en": "Browse", "zh_CN": "浏览"},
    "save_as": {"en": "Save As", "zh_CN": "另存为"},
    "play": {"en": "Play", "zh_CN": "播放"},
    "pause": {"en": "Pause", "zh_CN": "暂停"},
    "ready": {"en": "Ready", "zh_CN": "就绪"},
    "elapsed_idle": {"en": "Elapsed 00:00 · ETA --:--", "zh_CN": "已用时 00:00 · 预计 --:--"},
    "no_analysis": {"en": "No analysis yet", "zh_CN": "尚未分析"},
    "status_queued": {"en": "Queued", "zh_CN": "排队"},
    "status_copying": {"en": "Copying", "zh_CN": "正在复制"},
    "status_stitching": {"en": "Stitching SBS", "zh_CN": "正在拼接 SBS"},
    "status_copied": {"en": "Completed · Direct Copy", "zh_CN": "已完成 · 直接复制"},
    "status_completed": {"en": "Completed · SBS", "zh_CN": "已完成 · SBS"},
    "status_failed": {"en": "Failed", "zh_CN": "失败"},
    "stage_copy": {"en": "Copy to Computer", "zh_CN": "复制到电脑"},
    "stage_inspect": {"en": "Inspect Video Type", "zh_CN": "检查视频类型"},
    "stage_stitch": {"en": "Stitch SBS", "zh_CN": "拼接 SBS"},
    "stage_complete": {"en": "Completed", "zh_CN": "已完成"},
    "workflow_line": {"en": "{stage} {stage_percent}% · {name} · Total {overall_percent}%", "zh_CN": "{stage} {stage_percent}% · {name} · 总进度 {overall_percent}%"},
    "files_read": {"en": "Loaded {count} media items · JPEG / MP4 / HEIF / DNG / Legacy Stereo", "zh_CN": "已读取 {count} 个媒体项目 · JPEG / MP4 / HEIF / DNG / 旧版双目"},
    "legacy_stereo": {"en": "LEGACY 4-FILE → SBS", "zh_CN": "旧版四文件 → SBS"},
    "no_media": {"en": "No supported media files were found in DCIM/VR180.", "zh_CN": "DCIM/VR180 中没有找到支持的媒体文件。"},
    "export_complete": {"en": "Export complete: {count} files · {destination}", "zh_CN": "导出完成：{count} 个文件 · {destination}"},
    "export_complete_dialog": {"en": "Exported {count} files to:\n{destination}", "zh_CN": "已导出 {count} 个文件到：\n{destination}"},
    "export_failed": {"en": "Operation failed: {message}", "zh_CN": "操作失败：{message}"},
    "choose_export_folder": {"en": "Choose Export Folder", "zh_CN": "选择导出文件夹"},
    "no_export_folder": {"en": "No export folder was selected.", "zh_CN": "未选择导出文件夹。"},
    "adb_missing": {"en": "ADB was not found. Install Android Platform Tools or add adb.exe to PATH.", "zh_CN": "未找到 ADB。请安装 Android Platform Tools，或将 adb.exe 加入 PATH。"},
    "export_cancelled": {"en": "Export was cancelled.", "zh_CN": "文件导出已取消。"},
    "remote_export_failed": {"en": "Failed to export {remote_path}.", "zh_CN": "导出 {remote_path} 失败。"},
    "processor_name": {"en": "Qualcomm Snapdragon 8 Gen 2 · Dual CMOS", "zh_CN": "高通骁龙8GN2 · 双 CMOS"},
}


def ui_text(key: str, language: str, **values: object) -> str:
    entry = UI_TEXT.get(key)
    text = entry.get(language, entry.get("en", key)) if entry else key
    return text.format(**values) if values else text


def localize_device_error(message: str, language: str) -> str:
    if language == "zh_CN":
        return message
    if "未找到 ADB" in message:
        return ui_text("adb_missing", "en")
    if "未检测到 SLAM XCAM" in message:
        return "SLAM XCAM was not detected. Connect the camera and enable USB debugging."
    if "尚未授权" in message:
        return "The camera has not authorized USB debugging. Confirm the authorization on the camera."
    if "当前状态为" in message:
        return "The camera USB connection is not ready. Reconnect USB and try again."
    if "无法解析设备存储信息" in message:
        return "Unable to parse device storage information."
    if "无效的存储容量" in message:
        return "The camera returned an invalid storage capacity."
    if "ADB 命令执行失败" in message:
        return "The ADB command failed."
    return message


MINIMAL_THEME = """
QMainWindow, QWidget { background: #f3f3f3; color: #202020; font-family: "Segoe UI", "Microsoft YaHei UI", Arial, sans-serif; font-size: 13px; }
QLabel { background: transparent; }
QWidget#fieldRow, QWidget#controlRow { background: transparent; }
QFrame#studioHeader { background: #ffffff; border-bottom: 1px solid #d8d8d8; }
QLabel#appTitle { font-size: 18px; font-weight: 600; }
QLabel#mutedText { color: #707070; font-size: 12px; }
QLabel#logo { background: #202020; color: #ffffff; border-radius: 3px; font-size: 10px; font-weight: 700; }
QFrame#topMenuBar { background: #ffffff; border-bottom: 1px solid #dddddd; }
QToolButton#menuButton { min-height: 28px; max-height: 28px; border: none; border-radius: 2px; padding: 0 8px; background: transparent; font-weight: 500; }
QToolButton#menuButton::menu-indicator { image: none; width: 0; }
QToolButton#menuButton:hover, QToolButton#menuButton:pressed { background: #eeeeee; }
QToolButton { min-height: 32px; background: #ffffff; color: #202020; border: 1px solid #bdbdbd; border-radius: 3px; padding: 3px 10px; font-weight: 500; }
QToolButton:hover { background: #f4f4f4; border-color: #9b9b9b; }
QMenu { background: #ffffff; color: #202020; border: 1px solid #bcbcbc; padding: 3px; }
QMenu::item { padding: 6px 28px 6px 10px; border-radius: 2px; }
QMenu::item:selected { background: #e9e9e9; }
QFrame#pageTabs { background: #ffffff; border-bottom: 1px solid #d5d5d5; }
QPushButton[pageTab="true"] { min-height: 38px; padding: 0 18px; border: none; border-bottom: 2px solid transparent; border-radius: 0; background: transparent; color: #666666; font-weight: 500; }
QPushButton[pageTab="true"]:hover { background: #f6f6f6; color: #202020; }
QPushButton[pageTab="true"][active="true"] { background: transparent; color: #202020; border-bottom: 2px solid #202020; font-weight: 600; }
QFrame#mediaPanel, QFrame#previewPanel, QFrame#cameraSide, QFrame#cameraContent { background: #ffffff; border: 1px solid #d0d0d0; border-radius: 3px; }
QFrame#studioPage { background: #f3f3f3; border: none; }
QFrame#exportProgressPanel { background: #f7f7f7; border: 1px solid #d5d5d5; border-radius: 3px; }
QFrame#segmentedTabs { background: #ffffff; border: none; border-bottom: 1px solid #d5d5d5; border-radius: 0; }
QPushButton[leftTab="true"] { background: transparent; color: #6a6a6a; border: none; border-bottom: 2px solid transparent; border-radius: 0; padding: 7px 4px; font-size: 12px; }
QPushButton[leftTab="true"]:hover { color: #202020; background: #f7f7f7; }
QPushButton[leftTab="true"][active="true"] { background: transparent; color: #202020; border-bottom: 2px solid #202020; font-weight: 600; }
QPushButton { min-height: 32px; background: #252525; color: #ffffff; border: 1px solid #252525; border-radius: 3px; padding: 3px 11px; font-weight: 500; }
QPushButton:hover { background: #3c3c3c; border-color: #3c3c3c; }
QPushButton:disabled { background: #e4e4e4; color: #999999; border-color: #d8d8d8; }
QPushButton[secondary="true"] { background: #ffffff; color: #202020; border: 1px solid #bdbdbd; }
QPushButton[secondary="true"]:hover { background: #f2f2f2; border-color: #999999; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit, QListWidget { background: #ffffff; color: #202020; border: 1px solid #bdbdbd; border-radius: 3px; padding: 4px 8px; selection-background-color: #d7d7d7; selection-color: #111111; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus, QPlainTextEdit:focus { border-color: #5f5f5f; }
QComboBox::drop-down { border: none; width: 22px; }
QLabel#fieldLabel { color: #484848; font-size: 12px; font-weight: 500; }
QLabel#fixedAlgorithm, QLabel#infoBox, QLabel#matchBox { background: #f5f5f5; border: 1px solid #d4d4d4; border-radius: 3px; padding: 7px; color: #3b3b3b; }
QGroupBox { font-weight: 500; border: 1px solid #d0d0d0; border-radius: 3px; margin-top: 9px; padding: 8px; background: #ffffff; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #4a4a4a; }
QWidget#previewHeader { background: #fafafa; border-bottom: 1px solid #d3d3d3; }
QLabel#previewTitle { font-size: 15px; font-weight: 600; }
QFrame#previewViewport, #videoSurface, QLabel#previewSurface { background: #101010; color: #a7a7a7; border: none; }
QWidget#settingsPage { background: #ffffff; }
QProgressBar { min-height: 6px; max-height: 6px; border: none; border-radius: 0; background: #dddddd; }
QProgressBar::chunk { border-radius: 0; background: #353535; }
QProgressBar#deviceExportProgress { min-height: 18px; max-height: 18px; border-radius: 2px; color: #ffffff; font-weight: 600; text-align: center; }
QFrame#metricCard, QFrame#infoCard { background: #fafafa; border: 1px solid #d3d3d3; border-radius: 3px; }
QLabel#metricValue { font-size: 16px; font-weight: 600; }
QListWidget::item { border-bottom: 1px solid #e2e2e2; padding: 7px; }
QListWidget::item:selected { background: #e5e5e5; color: #202020; }
QSplitter::handle { background: #e2e2e2; width: 1px; }
QScrollBar:vertical { background: transparent; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background: #c2c2c2; min-height: 24px; border-radius: 3px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""


PIXEL_THEME = """
QMainWindow, QWidget { background: #151515; color: #d7d7d7; font-family: Consolas, "Microsoft YaHei", monospace; }
QLabel#appTitle { font-size: 22px; font-weight: 700; }
QLabel#mutedText { color: #929292; font-size: 12px; }
QLabel#logo { background: #242424; color: #d7d7d7; border: 2px solid #858585; border-radius: 0; font-size: 10px; font-weight: 700; }
QFrame#topMenuBar { border-bottom: 1px solid #666666; }
QToolButton#menuButton { min-height: 28px; max-height: 28px; border: none; border-right: 1px solid #666666; border-radius: 0; padding: 0 8px; background: #181818; font-weight: 700; }
QToolButton#menuButton::menu-indicator { image: none; width: 0; }
QToolButton#menuButton:hover, QToolButton#menuButton:pressed { background: #b8b8b8; color: #181818; }
QToolButton { min-height: 34px; background: #181818; color: #d7d7d7; border: 1px solid #858585; border-radius: 0; padding: 5px 12px; font-weight: 700; }
QToolButton:hover { background: #303030; }
QMenu { background: #202020; color: #d7d7d7; border: 1px solid #858585; padding: 4px; }
QMenu::item { padding: 7px 28px 7px 10px; }
QMenu::item:selected { background: #b8b8b8; color: #181818; }
QFrame#pageTabs { background: transparent; border-bottom: 2px solid #858585; }
QPushButton[pageTab="true"] { min-height: 36px; padding: 0 20px; border: 1px solid #858585; border-radius: 0; background: #181818; color: #929292; font-weight: 700; }
QPushButton[pageTab="true"][active="true"] { background: #b8b8b8; color: #181818; }
QFrame#mediaPanel, QFrame#previewPanel, QFrame#studioPage, QFrame#cameraSide, QFrame#cameraContent { background: #202020; border: 1px solid #858585; border-radius: 0; }
QFrame#exportProgressPanel { background: #252525; border: 1px solid #666666; border-radius: 0; }
QFrame#segmentedTabs { background: #181818; border: 1px solid #858585; border-radius: 0; }
QPushButton[leftTab="true"] { background: #181818; color: #929292; border: none; border-right: 1px solid #666666; border-radius: 0; padding: 8px; }
QPushButton[leftTab="true"][active="true"] { background: #b8b8b8; color: #181818; }
QPushButton { min-height: 34px; background: #181818; color: #d7d7d7; border: 1px solid #858585; border-radius: 0; padding: 5px 12px; font-weight: 700; }
QPushButton:hover { background: #303030; }
QPushButton:disabled { background: #252525; color: #666666; border-color: #555555; }
QPushButton[secondary="true"] { background: #181818; color: #c8c8c8; border: 1px solid #858585; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit, QListWidget { background: #181818; color: #d7d7d7; border: 1px solid #858585; border-radius: 0; padding: 5px 9px; }
QComboBox::drop-down { border: none; width: 24px; }
QLabel#fieldLabel { color: #cecece; font-size: 13px; font-weight: 700; }
QLabel#fixedAlgorithm, QLabel#infoBox, QLabel#matchBox { background: #292929; border: 1px solid #666666; border-radius: 0; padding: 8px; color: #bcbcbc; }
QGroupBox { font-weight: 700; border: 1px solid #858585; border-radius: 0; margin-top: 10px; padding: 10px; background: #202020; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QWidget#previewHeader { background: #202020; border-bottom: 1px solid #858585; }
QLabel#previewTitle { font-size: 17px; font-weight: 700; }
QFrame#previewViewport, #videoSurface, QLabel#previewSurface { background: #101010; color: #999999; border: none; }
QWidget#settingsPage { background: #202020; }
QProgressBar { min-height: 10px; max-height: 10px; border: 1px solid #666666; border-radius: 0; background: #181818; }
QProgressBar::chunk { background: #a8a8a8; }
QProgressBar#deviceExportProgress { min-height: 20px; max-height: 20px; color: #111111; font-weight: 700; text-align: center; }
QFrame#metricCard, QFrame#infoCard { background: #1a1a1a; border: 1px solid #666666; border-radius: 0; }
QLabel#metricValue { font-size: 17px; font-weight: 700; }
QListWidget::item { border-bottom: 1px solid #555555; padding: 8px; }
QListWidget::item:selected { background: #a8a8a8; color: #181818; }
"""


def resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2]))
    return base / relative


def app_path(relative: str) -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / relative
    return Path(relative).resolve()


def _parse_adb_devices(output: str) -> tuple[str, str]:
    devices: list[tuple[str, str]] = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("List of devices") or line.startswith("*"):
            continue
        fields = line.split()
        if len(fields) >= 2:
            devices.append((fields[0], fields[1]))
    for serial, state in devices:
        if state == "device":
            return serial, state
    if devices:
        serial, state = devices[0]
        if state == "unauthorized":
            raise RuntimeError(f"设备 {serial} 尚未授权，请在相机端确认 USB 调试授权。")
        raise RuntimeError(f"设备 {serial} 当前状态为 {state}，请重新连接 USB。")
    raise RuntimeError("未检测到 SLAM XCAM，请连接相机并确认 USB 调试已启用。")


def _parse_storage_stat(output: str) -> tuple[int, int, int]:
    fields = output.strip().split()
    if len(fields) < 3:
        raise ValueError(f"无法解析设备存储信息: {output.strip() or 'empty output'}")
    block_size, block_count, available_blocks = (int(value) for value in fields[-3:])
    total_bytes = block_size * block_count
    available_bytes = block_size * available_blocks
    if total_bytes <= 0 or not 0 <= available_bytes <= total_bytes:
        raise ValueError("设备返回了无效的存储容量。")
    return total_bytes, total_bytes - available_bytes, available_bytes


def _format_gib(byte_count: int) -> str:
    return f"{byte_count / (1024 ** 3):.1f} GB"


def _format_file_size(byte_count: int) -> str:
    units = ("B", "KB", "MB", "GB", "TB")
    size = float(max(0, byte_count))
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{byte_count} B"


def _parse_device_media(output: str) -> list[dict[str, object]]:
    physical_files: list[dict[str, object]] = []
    for raw_line in output.splitlines():
        fields = raw_line.strip().rsplit("|", 2)
        if len(fields) != 3:
            continue
        remote_path, size_text, modified_text = fields
        extension = Path(remote_path).suffix.lower()
        if extension not in SUPPORTED_DEVICE_MEDIA_EXTENSIONS | LEGACY_DEVICE_COMPONENT_EXTENSIONS:
            continue
        try:
            size_bytes = int(size_text)
            modified_epoch = int(modified_text)
        except ValueError:
            continue
        physical_files.append(
            {
                "remote_path": remote_path,
                "name": remote_path.rsplit("/", 1)[-1],
                "extension": extension,
                "size_bytes": size_bytes,
                "modified_epoch": modified_epoch,
            }
        )
    legacy_parts: dict[str, dict[str, dict[str, object]]] = {}
    ordinary: list[dict[str, object]] = []
    suffix_roles = {
        "_L.mp4": "left",
        "_R.mp4": "right",
        "_A.m4a": "audio",
        "_sync.json": "sync",
    }
    for file_info in physical_files:
        name = str(file_info["name"])
        match = next(
            ((suffix, role) for suffix, role in suffix_roles.items() if name.endswith(suffix)),
            None,
        )
        if match is None:
            if str(file_info["extension"]) in SUPPORTED_DEVICE_MEDIA_EXTENSIONS:
                ordinary.append(file_info)
            continue
        suffix, role = match
        legacy_parts.setdefault(name[: -len(suffix)], {})[role] = file_info

    for base, components in legacy_parts.items():
        if not all(role in components for role in ("left", "right", "audio", "sync")):
            continue
        values = list(components.values())
        ordinary.append(
            {
                "remote_path": components["sync"]["remote_path"],
                "name": f"{base}_VR180.mp4",
                "extension": ".mp4",
                "kind": "legacy_stereo_session",
                "components": components,
                "size_bytes": sum(int(item["size_bytes"]) for item in values),
                "modified_epoch": max(int(item["modified_epoch"]) for item in values),
            }
        )
    return sorted(
        ordinary,
        key=lambda item: (int(item["modified_epoch"]), str(item["name"])),
        reverse=True,
    )


def _find_adb() -> str | None:
    candidates: list[Path] = []
    bundle_root = getattr(sys, "_MEIPASS", None)
    if bundle_root:
        candidates.append(Path(bundle_root) / "tools" / "platform-tools" / "adb.exe")
        candidates.append(Path(bundle_root) / "platform-tools" / "adb.exe")
    if getattr(sys, "frozen", False):
        candidates.append(Path(sys.executable).resolve().parent / "tools" / "platform-tools" / "adb.exe")
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        candidates.append(Path(local_app_data) / "Android" / "Sdk" / "platform-tools" / "adb.exe")
    candidates.extend(Path(path) for path in (find_tool("adb"),) if path)
    return str(next((path for path in candidates if path.is_file()), "")) or None


@lru_cache(maxsize=4)
def _adb_supports_no_compression(adb: str) -> bool:
    completed = subprocess.run(
        [adb, "version"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
        **hidden_subprocess_kwargs(),
    )
    return "version 1.0.41" in (completed.stdout + completed.stderr).lower()


def _run_adb(arguments: list[str], timeout_s: float = 8.0) -> str:
    adb = _find_adb()
    if not adb:
        raise RuntimeError("未找到 ADB。请安装 Android Platform Tools，或将 adb.exe 加入 PATH。")
    completed = subprocess.run(
        [adb, *arguments],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout_s,
        check=False,
        **hidden_subprocess_kwargs(),
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(detail or f"ADB 命令执行失败，退出码 {completed.returncode}。")
    return completed.stdout


def probe_slam_xcam_device() -> dict[str, object]:
    serial, _state = _parse_adb_devices(_run_adb(["devices"]))
    selector = ["-s", serial, "shell"]
    # adb shell joins separate host arguments without preserving the quoted
    # stat format. Send one remote command so Android receives it as one value.
    storage = _run_adb([*selector, "stat -f -c '%S %b %a' /sdcard"])
    total_bytes, used_bytes, available_bytes = _parse_storage_stat(storage)
    soc_model = _run_adb([*selector, "getprop", "ro.soc.model"]).strip()
    return {
        "serial": serial,
        "device_name": "SLAM XCAM",
        "processor": "高通骁龙8GN2 DUAL CMOS",
        "soc_model": soc_model,
        "total_bytes": total_bytes,
        "used_bytes": used_bytes,
        "available_bytes": available_bytes,
    }


def list_device_media(serial: str) -> list[dict[str, object]]:
    command = (
        f"find {DEVICE_MEDIA_ROOT} -maxdepth 1 -type f "
        "-exec stat -c '%n|%s|%Y' {} +"
    )
    output = _run_adb(
        ["-s", serial, "shell", command],
        timeout_s=30.0,
    )
    return _parse_device_media(output)


class DeviceInfoWorker(QObject):
    finished = Signal(dict)
    failed = Signal(str)

    def __init__(self, language: str = "en") -> None:
        super().__init__()
        self.language = "zh_CN" if language == "zh_CN" else "en"

    def run(self) -> None:
        try:
            self.finished.emit(probe_slam_xcam_device())
        except Exception as exc:
            self.failed.emit(localize_device_error(str(exc), self.language))


class DeviceFileWorker(QObject):
    listed = Signal(list)
    exported = Signal(dict)
    failed = Signal(str)
    file_status = Signal(str, str, str)
    workflow_progress = Signal(int, int, str, str)

    def __init__(
        self,
        action: str,
        serial: str,
        files: list[dict[str, object]] | None = None,
        destination: str = "",
        language: str = "en",
    ) -> None:
        super().__init__()
        self.action = action
        self.serial = serial
        self.files = files or []
        self.destination = Path(destination) if destination else None
        self.language = "zh_CN" if language == "zh_CN" else "en"

    def run(self) -> None:
        try:
            if self.action == "list":
                self.listed.emit(list_device_media(self.serial))
                return
            self.exported.emit(self._export())
        except Exception as exc:
            self.failed.emit(str(exc))

    def _export(self) -> dict[str, object]:
        if self.destination is None:
            raise ValueError(ui_text("no_export_folder", self.language))
        self.destination.mkdir(parents=True, exist_ok=True)
        exported_paths: list[str] = []
        converted_sources: list[dict[str, str]] = []
        total = len(self.files)
        for index, file_info in enumerate(self.files, start=1):
            remote_path = str(file_info["remote_path"])
            if file_info.get("kind") == "legacy_stereo_session":
                try:
                    output_path, encoder = self._export_legacy_session(file_info, index, total)
                    exported_paths.append(str(output_path))
                    converted_sources.append(
                        {
                            "source": str(file_info["name"]),
                            "output": output_path.name,
                            "encoder": encoder,
                        }
                    )
                    self._emit_workflow(index, total, 100, "stage_complete", str(file_info["name"]))
                except Exception as exc:
                    self.file_status.emit(remote_path, "failed", str(exc))
                    raise
                continue
            local_path = self._unique_destination(self.destination / str(file_info["name"]))
            file_name = str(file_info["name"])
            expected_size = int(file_info.get("size_bytes") or 0)
            is_video = local_path.suffix.lower() == ".mp4"
            copy_weight = 45 if is_video else 100
            try:
                self.file_status.emit(remote_path, "copying", str(local_path))
                self._emit_workflow(index, total, 0, "stage_copy", file_name)
                self._pull(
                    remote_path,
                    local_path,
                    expected_size,
                    lambda percent: self._emit_workflow(
                        index,
                        total,
                        percent * copy_weight / 100,
                        "stage_copy",
                        file_name,
                    ),
                )
                exported_path = local_path
                if is_video:
                    self._emit_workflow(index, total, 48, "stage_inspect", file_name)
                source = inspect_slam_source(local_path) if is_video else None
                if source is not None:
                    validate_processed_source(source)
                    output_path = self._unique_destination(default_sbs_export_path(local_path))
                    self.file_status.emit(remote_path, "stitching", str(output_path))
                    self._emit_workflow(index, total, 50, "stage_stitch", file_name)
                    encoder = export_slam_source(
                        source,
                        output_path,
                        progress_callback=lambda frame, frame_count: self._emit_workflow(
                            index,
                            total,
                            50 + 50 * min(frame / frame_count, 1) if frame_count else 50,
                            "stage_stitch",
                            file_name,
                        ),
                    )
                    local_path.unlink()
                    exported_path = output_path
                    converted_sources.append(
                        {"source": local_path.name, "output": output_path.name, "encoder": encoder}
                    )
                    self.file_status.emit(remote_path, "completed", str(output_path))
                else:
                    self.file_status.emit(remote_path, "copied", str(local_path))
                exported_paths.append(str(exported_path))
                self._emit_workflow(index, total, 100, "stage_complete", file_name)
            except Exception as exc:
                self.file_status.emit(remote_path, "failed", str(exc))
                raise
        return {
            "destination": str(self.destination),
            "paths": exported_paths,
            "converted_sources": converted_sources,
        }

    def _export_legacy_session(
        self,
        file_info: dict[str, object],
        index: int,
        total: int,
    ) -> tuple[Path, str]:
        if self.destination is None:
            raise ValueError(ui_text("no_export_folder", self.language))
        components = file_info.get("components")
        if not isinstance(components, dict):
            raise ValueError("Legacy stereo session components are missing.")
        remote_key = str(file_info["remote_path"])
        file_name = str(file_info["name"])
        self.file_status.emit(remote_key, "copying", "")
        self._emit_workflow(index, total, 0, "stage_copy", file_name)
        component_order = ("sync", "audio", "left", "right")
        component_values = [components[role] for role in component_order]
        expected_total = sum(int(item.get("size_bytes") or 0) for item in component_values)
        copied_before = 0
        with tempfile.TemporaryDirectory(prefix="slam_xcam_legacy_", dir=self.destination) as temp_dir:
            local_parts: dict[str, Path] = {}
            for role, component in zip(component_order, component_values):
                if not isinstance(component, dict):
                    raise ValueError(f"Legacy stereo session is missing {role} metadata.")
                expected_size = int(component.get("size_bytes") or 0)
                local_path = Path(temp_dir) / str(component["name"])

                def component_progress(percent: float, before: int = copied_before, size: int = expected_size) -> None:
                    if expected_total > 0:
                        aggregate = (before + size * percent / 100) * 100 / expected_total
                    else:
                        aggregate = percent
                    self._emit_workflow(index, total, aggregate * 0.45, "stage_copy", file_name)

                self._pull(str(component["remote_path"]), local_path, expected_size, component_progress)
                local_parts[role] = local_path
                copied_before += expected_size

            self._emit_workflow(index, total, 48, "stage_inspect", file_name)
            source = inspect_legacy_slam_source(
                local_parts["sync"],
                local_parts["left"],
                local_parts["right"],
                local_parts["audio"],
            )
            suggested = default_legacy_sbs_export_path(local_parts["sync"]).name
            output_path = self._unique_destination(self.destination / suggested)
            self.file_status.emit(remote_key, "stitching", str(output_path))
            self._emit_workflow(index, total, 50, "stage_stitch", file_name)
            encoder = export_legacy_slam_source(
                source,
                output_path,
                progress_callback=lambda frame, frame_count: self._emit_workflow(
                    index,
                    total,
                    50 + 50 * min(frame / frame_count, 1) if frame_count else 50,
                    "stage_stitch",
                    file_name,
                ),
            )
        self.file_status.emit(remote_key, "completed", str(output_path))
        return output_path, encoder

    def _emit_workflow(
        self,
        index: int,
        total: int,
        file_percent: float,
        stage: str,
        file_name: str,
    ) -> None:
        stage_percent = max(0, min(int(round(file_percent)), 100))
        overall = int(round(((index - 1) + stage_percent / 100) * 100 / total)) if total else 0
        self.workflow_progress.emit(overall, stage_percent, stage, file_name)

    @staticmethod
    def _unique_destination(path: Path) -> Path:
        if not path.exists():
            return path
        counter = 1
        while True:
            candidate = path.with_name(f"{path.stem} ({counter}){path.suffix}")
            if not candidate.exists():
                return candidate
            counter += 1

    def _pull(self, remote_path: str, local_path: Path, expected_size: int, progress_callback) -> None:
        adb = _find_adb()
        if not adb:
            raise RuntimeError(ui_text("adb_missing", self.language))
        pull_options = ["-Z"] if _adb_supports_no_compression(adb) else []
        process = subprocess.Popen(
            [adb, "-s", self.serial, "pull", *pull_options, remote_path, str(local_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            **hidden_subprocess_kwargs(),
        )
        while process.poll() is None:
            if QThread.currentThread().isInterruptionRequested():
                process.terminate()
                process.wait(timeout=5)
                raise RuntimeError(ui_text("export_cancelled", self.language))
            if expected_size > 0 and local_path.exists():
                progress_callback(min(local_path.stat().st_size * 100 / expected_size, 99))
            time.sleep(0.1)
        stdout, stderr = process.communicate()
        if process.returncode != 0:
            detail = (stderr or stdout).strip()
            raise RuntimeError(
                detail or ui_text("remote_export_failed", self.language, remote_path=remote_path)
            )
        progress_callback(100)


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
        if not str(video_path):
            raise ValueError("Please select an SBS video, or switch to separate left/right mode and select a left video.")
        source = inspect_slam_source(video_path) if mode == "sbs" else None
        if source is not None:
            validate_processed_source(source)
            report_path = Path(self.payload["report"])
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report = {
                "video": {
                    "path": str(video_path),
                    "container_type": source.manifest.get("containerType"),
                    "layout": "dual_track_vr180_fisheye",
                    "left_video_ordinal": source.left_video_ordinal,
                    "right_video_ordinal": source.right_video_ordinal,
                    "audio_ordinal": source.audio_ordinal,
                    "eye_width": source.eye_width,
                    "eye_height": source.eye_height,
                    "output_width": source.output_width,
                    "output_height": source.output_height,
                    "requested_fps": source.requested_fps,
                },
                "source_manifest": source.manifest,
            }
            self._emit_progress(80, "Writing source-container report")
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            self._emit_progress(100, "Analysis complete")
            return (
                "Source analysis complete.\n"
                f"Source: two {source.eye_width}x{source.eye_height} video tracks at "
                f"{source.requested_fps:g}fps\n"
                f"Export: {source.output_width}x{source.output_height} VR180 SBS\n"
                "K/D distortion correction and stereo X/Y alignment are already applied; "
                "Studio will not apply them again.\n"
                f"Report: {report_path}\n"
            )
        imu_path = Path(self.payload["imu"])
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
        source_path = Path(self.payload["sbs"]) if mode == "sbs" else None
        source = inspect_slam_source(source_path) if source_path is not None else None
        if source is not None:
            self._emit_progress(5, "Validating SLAM XCAM 8K50 source container")
            validate_processed_source(source)
            self._emit_progress(10, "Exporting synchronized 8K50 VR180 SBS video")
            encoder = export_slam_source(source, output)
            self._emit_progress(100, "8K50 VR180 export complete")
            return (
                "8K50 source export complete.\n"
                f"Output video: {output}\n"
                f"Output: {source.output_width}x{source.output_height} at "
                f"{source.requested_fps:g}fps\n"
                f"HEVC encoder: {encoder}\n"
                "Existing K/D and stereo X/Y correction were preserved without being applied twice.\n"
            )
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
        self.setWindowTitle("SLAM XCAM Studio")
        self.setWindowIcon(
            QIcon(str(resource_path("assets/branding/slam-xcam-studio-stacked-dark.png")))
        )
        self.resize(1280, 780)
        self.setMinimumSize(760, 520)
        settings_path = os.environ.get("SLAM_XCAM_SETTINGS_PATH")
        self.settings = (
            QSettings(settings_path, QSettings.Format.IniFormat)
            if settings_path
            else QSettings("SLAM XCAM", "SLAM XCAM Studio")
        )
        self.theme_name = str(self.settings.value("ui/theme", "minimal"))
        self.language = str(self.settings.value("ui/language", "en"))
        if self.language not in {"en", "zh_CN"}:
            self.language = "en"
        self.thread: QThread | None = None
        self.worker: Worker | None = None
        self.device_thread: QThread | None = None
        self.device_worker: DeviceInfoWorker | None = None
        self.device_file_thread: QThread | None = None
        self.device_file_worker: DeviceFileWorker | None = None
        self.device_file_action = ""
        self.current_device_serial = ""
        self.device_refresh_button: QPushButton | None = None
        self.device_files_refresh_button: QPushButton | None = None
        self.device_button: QPushButton | None = None
        self.device_files_button: QToolButton | None = None
        self.device_name_value: QLabel | None = None
        self.device_storage_value: QLabel | None = None
        self.device_processor_value: QLabel | None = None
        self.device_status_label: QLabel | None = None
        self.device_files_status_label: QLabel | None = None
        self.device_export_progress: QProgressBar | None = None
        self.device_export_panel: QFrame | None = None
        self.left_tab_buttons: list[QPushButton] = []
        self.left_stack: QStackedWidget | None = None
        self.page_tab_buttons: list[QPushButton] = []
        self.page_stack: QStackedWidget | None = None
        self.device_content_stack: QStackedWidget | None = None
        self.device_file_list: QListWidget | None = None
        self.export_device_files_button: QPushButton | None = None
        self.brand_logo: QLabel | None = None

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
            field.setFixedHeight(36)

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
        self._apply_theme(self.theme_name, persist=False)
        self._apply_language(self.language, persist=False)
        self._apply_model_defaults()
        self._refresh_algorithm_info()
        self._refresh_default_outputs()
        if self.sbs.text():
            self._auto_match_imu(Path(self.sbs.text()), force=False)
            self._set_preview_video(Path(self.sbs.text()), load_thumbnail=False)
        self._refresh_preview_details()
        QTimer.singleShot(250, self.refresh_device_info)

    def _build(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header_bar = QFrame()
        header_bar.setObjectName("studioHeader")
        header_bar.setFixedHeight(52)
        header = QHBoxLayout(header_bar)
        header.setContentsMargins(16, 0, 16, 0)
        header.setSpacing(10)
        self.brand_logo = QLabel()
        self.brand_logo.setObjectName("brandLogo")
        self.brand_logo.setFixedSize(300, 40)
        self.brand_logo.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        header.addWidget(self.brand_logo)
        header.addStretch(1)
        root.addWidget(header_bar)

        root.addWidget(self._menu_row())
        root.addWidget(self._page_tabs())

        self.page_stack = QStackedWidget()
        stabilization_page = QWidget()
        stabilization_layout = QVBoxLayout(stabilization_page)
        stabilization_layout.setContentsMargins(12, 12, 12, 10)
        stabilization_layout.setSpacing(9)
        workspace = QSplitter()
        workspace.setChildrenCollapsible(False)
        workspace.setHandleWidth(7)
        workspace.addWidget(self._media_panel())
        workspace.addWidget(self._preview_panel())
        workspace.setSizes([350, 930])
        stabilization_layout.addWidget(workspace, 1)

        bottom = QGroupBox("Job Log")
        bottom.setMaximumHeight(152)
        bottom_layout = QVBoxLayout(bottom)
        bottom_layout.setContentsMargins(8, 8, 8, 8)
        self.log.setMinimumHeight(70)
        self.log.setMaximumHeight(110)
        self.log.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bottom_layout.addWidget(self.log)
        stabilization_layout.addWidget(bottom)
        self.page_stack.addWidget(stabilization_page)
        self.page_stack.addWidget(self._camera_page())
        self.page_stack.addWidget(self._editor_page())
        root.addWidget(self.page_stack, 1)
        self.setCentralWidget(central)

    def _menu_row(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topMenuBar")
        bar.setFixedHeight(36)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(0)
        self.menu_button = QToolButton()
        self.menu_button.setObjectName("menuButton")
        self.menu_button.setText("菜单  ▾")
        self.menu_button.setMinimumWidth(108)
        self.menu_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        self.menu_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self.menu_button)
        self.ui_menu = menu.addMenu("UI 设置")
        theme_group = QActionGroup(self)
        theme_group.setExclusive(True)
        self.minimal_theme_action = QAction("极简风", self, checkable=True)
        self.pixel_theme_action = QAction("像素风", self, checkable=True)
        theme_group.addAction(self.minimal_theme_action)
        theme_group.addAction(self.pixel_theme_action)
        self.ui_menu.addActions([self.minimal_theme_action, self.pixel_theme_action])
        self.minimal_theme_action.triggered.connect(lambda: self._apply_theme("minimal"))
        self.pixel_theme_action.triggered.connect(lambda: self._apply_theme("pixel"))
        self.language_menu = menu.addMenu("语言设置")
        language_group = QActionGroup(self)
        language_group.setExclusive(True)
        self.english_action = QAction("English", self, checkable=True)
        self.chinese_action = QAction("简体中文", self, checkable=True)
        language_group.addAction(self.english_action)
        language_group.addAction(self.chinese_action)
        self.language_menu.addActions([self.english_action, self.chinese_action])
        self.english_action.triggered.connect(lambda: self._apply_language("en"))
        self.chinese_action.triggered.connect(lambda: self._apply_language("zh_CN"))
        self.menu_button.setMenu(menu)
        layout.addWidget(self.menu_button, 0, Qt.AlignmentFlag.AlignVCenter)
        layout.addStretch(1)
        return bar

    def _page_tabs(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("pageTabs")
        bar.setFixedHeight(42)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 0, 12, 0)
        layout.setSpacing(2)
        for index, label in enumerate(["防抖处理", "相机管理", "剪辑"]):
            button = QPushButton(label)
            button.setProperty("pageTab", True)
            button.setProperty("active", index == 0)
            button.clicked.connect(lambda checked=False, i=index: self._switch_page(i))
            self.page_tab_buttons.append(button)
            layout.addWidget(button)
        layout.addStretch(1)
        return bar

    def _switch_page(self, index: int) -> None:
        if self.page_stack is not None:
            self.page_stack.setCurrentIndex(index)
        for button_index, button in enumerate(self.page_tab_buttons):
            button.setProperty("active", button_index == index)
            button.style().unpolish(button)
            button.style().polish(button)

    def _camera_page(self) -> QWidget:
        page = QFrame()
        page.setObjectName("studioPage")
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(20, 20, 20, 20)
        page_layout.setSpacing(16)
        header = QHBoxLayout()
        header_text = QVBoxLayout()
        heading = QLabel("相机管理")
        heading.setObjectName("appTitle")
        description = QLabel("连接并识别 SLAM XCAM 的设备与存储信息。")
        description.setObjectName("mutedText")
        header_text.addWidget(heading)
        header_text.addWidget(description)
        header.addLayout(header_text)
        header.addStretch(1)
        self.device_refresh_button = QPushButton("刷新设备")
        self.device_refresh_button.setProperty("secondary", True)
        self.device_refresh_button.clicked.connect(self.refresh_device_info)
        header.addWidget(self.device_refresh_button)
        page_layout.addLayout(header)

        content = QHBoxLayout()
        content.setSpacing(16)
        side = QFrame()
        side.setObjectName("cameraSide")
        side.setFixedWidth(280)
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(14, 14, 14, 14)
        side_layout.setSpacing(10)
        side_layout.addWidget(QLabel("设备"))
        self.device_button = QPushButton("正在检测设备…")
        self.device_button.setProperty("secondary", True)
        self.device_button.setMinimumHeight(84)
        self.device_button.clicked.connect(lambda: self.device_content_stack.setCurrentIndex(0) if self.device_content_stack else None)
        side_layout.addWidget(self.device_button)
        self.device_files_button = QToolButton()
        self.device_files_button.setText("设备文件")
        self.device_files_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.device_files_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DirIcon))
        self.device_files_button.setIconSize(QSize(24, 24))
        self.device_files_button.setMinimumHeight(50)
        self.device_files_button.setEnabled(False)
        self.device_files_button.clicked.connect(self._show_device_files)
        side_layout.addWidget(self.device_files_button)
        side_layout.addStretch(1)
        content.addWidget(side)

        self.device_content_stack = QStackedWidget()
        self.device_content_stack.addWidget(self._camera_info_view())
        self.device_content_stack.addWidget(self._camera_files_view())
        content.addWidget(self.device_content_stack, 1)
        page_layout.addLayout(content, 1)
        return page

    def _camera_info_view(self) -> QWidget:
        view = QFrame()
        view.setObjectName("cameraContent")
        layout = QVBoxLayout(view)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        title = QLabel("SLAM XCAM")
        title.setObjectName("appTitle")
        layout.addWidget(title)
        self.device_status_label = QLabel("正在检测 USB 设备…")
        self.device_status_label.setObjectName("mutedText")
        layout.addWidget(self.device_status_label)
        metrics = QHBoxLayout()
        metrics.setSpacing(10)
        self.device_name_value = QLabel("--")
        self.device_storage_value = QLabel("--")
        self.device_processor_value = QLabel("--")
        metrics.addWidget(self._metric_card("设备", self.device_name_value), 1)
        metrics.addWidget(self._metric_card("存储空间", self.device_storage_value), 1)
        metrics.addWidget(self._metric_card("处理器型号", self.device_processor_value), 1)
        layout.addLayout(metrics)
        layout.addStretch(1)
        return view

    def _metric_card(self, label: str, value: str | QLabel) -> QWidget:
        card = QFrame()
        card.setObjectName("metricCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 10)
        caption = QLabel(label)
        caption.setObjectName("mutedText")
        result = value if isinstance(value, QLabel) else QLabel(value)
        result.setObjectName("metricValue")
        result.setWordWrap(True)
        layout.addWidget(caption)
        layout.addWidget(result)
        return card

    def refresh_device_info(self) -> None:
        if self.device_thread is not None:
            try:
                if self.device_thread.isRunning():
                    return
            except RuntimeError:
                self.device_thread = None
                self.device_worker = None
        if self.device_refresh_button is not None:
            self.device_refresh_button.setEnabled(False)
            self.device_refresh_button.setText(self._tr("detecting"))
        if self.device_status_label is not None:
            self.device_status_label.setText(self._tr("detecting_usb_xcam"))
        if self.device_button is not None:
            self.device_button.setText(self._tr("detecting_device"))
        if self.device_files_button is not None:
            self.device_files_button.setEnabled(False)

        thread = QThread(self)
        worker = DeviceInfoWorker(self.language)
        self.device_thread = thread
        self.device_worker = worker
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(self._device_info_ready)
        worker.failed.connect(self._device_info_failed)
        worker.finished.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(self._device_thread_finished)
        thread.finished.connect(thread.deleteLater)
        thread.start()

    def _device_info_ready(self, info: dict[str, object]) -> None:
        serial = str(info["serial"])
        if self.current_device_serial and self.current_device_serial != serial:
            self._clear_device_files()
        self.current_device_serial = serial
        total = int(info["total_bytes"])
        available = int(info["available_bytes"])
        if self.device_name_value is not None:
            self.device_name_value.setText(str(info["device_name"]))
        if self.device_storage_value is not None:
            self.device_storage_value.setText(
                self._tr(
                    "available_storage",
                    available=_format_gib(available),
                    total=_format_gib(total),
                )
            )
        if self.device_processor_value is not None:
            self.device_processor_value.setText(self._tr("processor_name"))
        if self.device_status_label is not None:
            self.device_status_label.setText(self._tr("connected_serial", serial=serial))
        if self.device_button is not None:
            self.device_button.setText(self._tr("device_button_connected", serial=serial))
        if self.device_files_button is not None:
            self.device_files_button.setEnabled(True)
        if self.device_files_refresh_button is not None:
            self.device_files_refresh_button.setEnabled(True)

    def _device_info_failed(self, message: str) -> None:
        self.current_device_serial = ""
        self._clear_device_files()
        if self.device_name_value is not None:
            self.device_name_value.setText(self._tr("not_detected"))
        if self.device_storage_value is not None:
            self.device_storage_value.setText("--")
        if self.device_processor_value is not None:
            self.device_processor_value.setText("--")
        if self.device_status_label is not None:
            self.device_status_label.setText(message)
        if self.device_button is not None:
            self.device_button.setText(self._tr("device_not_detected"))
        if self.device_files_refresh_button is not None:
            self.device_files_refresh_button.setEnabled(False)

    def _device_thread_finished(self) -> None:
        self.device_thread = None
        self.device_worker = None
        if self.device_refresh_button is not None:
            self.device_refresh_button.setEnabled(True)
            self.device_refresh_button.setText(self._tr("refresh_device"))

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        for thread in (self.device_file_thread, self.device_thread, self.thread):
            if thread is None:
                continue
            try:
                if thread.isRunning():
                    thread.requestInterruption()
                    thread.quit()
                    thread.wait(3000)
            except RuntimeError:
                pass
        super().closeEvent(event)

    def _camera_files_view(self) -> QWidget:
        view = QFrame()
        view.setObjectName("cameraContent")
        layout = QVBoxLayout(view)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)
        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("设备文件")
        title.setObjectName("appTitle")
        path = QLabel(DEVICE_MEDIA_ROOT)
        path.setObjectName("mutedText")
        title_box.addWidget(title)
        title_box.addWidget(path)
        header.addLayout(title_box)
        header.addStretch(1)
        self.device_files_refresh_button = QPushButton("重新读取")
        self.device_files_refresh_button.setProperty("secondary", True)
        self.device_files_refresh_button.setEnabled(False)
        self.device_files_refresh_button.clicked.connect(self._refresh_device_file_ui)
        self.export_device_files_button = QPushButton("导出所选文件")
        self.export_device_files_button.setEnabled(False)
        self.export_device_files_button.clicked.connect(self._export_device_files_ui)
        header.addWidget(self.device_files_refresh_button)
        header.addWidget(self.export_device_files_button)
        layout.addLayout(header)
        self.device_export_panel = QFrame()
        self.device_export_panel.setObjectName("exportProgressPanel")
        progress_layout = QVBoxLayout(self.device_export_panel)
        progress_layout.setContentsMargins(16, 13, 16, 14)
        progress_layout.setSpacing(10)
        self.device_files_status_label = QLabel("连接设备后读取 JPEG、MP4、HEIF 和 DNG 文件。")
        self.device_files_status_label.setObjectName("mutedText")
        progress_layout.addWidget(self.device_files_status_label)
        self.device_export_progress = QProgressBar()
        self.device_export_progress.setObjectName("deviceExportProgress")
        self.device_export_progress.setRange(0, 100)
        self.device_export_progress.setValue(0)
        self.device_export_progress.setTextVisible(True)
        progress_layout.addWidget(self.device_export_progress)
        layout.addWidget(self.device_export_panel)
        self.device_file_list = QListWidget()
        self.device_file_list.setAlternatingRowColors(False)
        self.device_file_list.itemChanged.connect(self._update_device_file_selection)
        layout.addWidget(self.device_file_list, 1)
        return view

    def _show_device_files(self) -> None:
        if self.device_content_stack is not None:
            self.device_content_stack.setCurrentIndex(1)
        if self.device_file_list is not None and self.device_file_list.count() == 0:
            self._refresh_device_file_ui()

    def _refresh_device_file_ui(self) -> None:
        if not self.current_device_serial:
            QMessageBox.warning(self, self._tr("device_files"), self._tr("connect_first"))
            return
        self._start_device_file_job("list")

    def _start_device_file_job(
        self,
        action: str,
        files: list[dict[str, object]] | None = None,
        destination: str = "",
    ) -> None:
        if self.device_file_thread is not None:
            try:
                if self.device_file_thread.isRunning():
                    return
            except RuntimeError:
                self.device_file_thread = None
                self.device_file_worker = None
        self.device_file_action = action
        if self.device_export_progress is not None:
            self.device_export_progress.setVisible(action == "export")
            self.device_export_progress.setFormat("%p%")
            self.device_export_progress.setValue(0)
        if self.device_files_refresh_button is not None:
            self.device_files_refresh_button.setEnabled(False)
        if self.export_device_files_button is not None:
            self.export_device_files_button.setEnabled(False)
        if self.device_file_list is not None:
            self.device_file_list.setEnabled(False)
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(
                self._tr("reading_media") if action == "list" else self._tr("preparing_export")
            )

        thread = QThread(self)
        worker = DeviceFileWorker(
            action,
            self.current_device_serial,
            files,
            destination,
            self.language,
        )
        self.device_file_thread = thread
        self.device_file_worker = worker
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.listed.connect(self._device_files_ready)
        worker.exported.connect(self._device_files_exported)
        worker.failed.connect(self._device_file_failed)
        worker.file_status.connect(self._device_file_status)
        worker.workflow_progress.connect(self._device_workflow_progress)
        worker.listed.connect(thread.quit)
        worker.exported.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(self._device_file_thread_finished)
        thread.finished.connect(thread.deleteLater)
        thread.start()

    def _device_files_ready(self, files: list[dict[str, object]]) -> None:
        if self.device_file_list is None:
            return
        self.device_file_list.blockSignals(True)
        self.device_file_list.clear()
        for file_info in files:
            extension = (
                self._tr("legacy_stereo")
                if file_info.get("kind") == "legacy_stereo_session"
                else str(file_info["extension"]).lstrip(".").upper()
            )
            if extension in {"JPG", "JPEG"}:
                extension = "JPEG"
            modified = time.strftime("%Y-%m-%d %H:%M", time.localtime(int(file_info["modified_epoch"])))
            detail = f"{extension} · {_format_file_size(int(file_info['size_bytes']))} · {modified}"
            item = QListWidgetItem(f"{file_info['name']}\n{detail}")
            item.setData(Qt.ItemDataRole.UserRole, file_info)
            item.setData(Qt.ItemDataRole.UserRole + 1, detail)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Unchecked)
            item.setSizeHint(QSize(0, 54))
            self.device_file_list.addItem(item)
        self.device_file_list.blockSignals(False)
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(
                self._tr("files_read", count=len(files))
                if files
                else self._tr("no_media")
            )
        if self.device_files_button is not None:
            self.device_files_button.setText(f"{self._tr('device_files')}    {len(files)}")
        self._update_device_file_selection()

    def _clear_device_files(self) -> None:
        if self.device_file_list is not None:
            self.device_file_list.clear()
        if self.device_files_button is not None:
            self.device_files_button.setText(self._tr("device_files"))
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(self._tr("media_help"))
        self._update_device_file_selection()

    def _update_device_file_selection(self, _item: QListWidgetItem | None = None) -> None:
        if self.device_file_list is None or self.export_device_files_button is None:
            return
        selected = sum(
            self.device_file_list.item(index).checkState() == Qt.CheckState.Checked
            for index in range(self.device_file_list.count())
        )
        self.export_device_files_button.setEnabled(selected > 0)
        label = self._tr("export_selected")
        self.export_device_files_button.setText(f"{label} ({selected})" if selected else label)

    def _export_device_files_ui(self) -> None:
        if not self.current_device_serial or self.device_file_list is None:
            return
        selected: list[dict[str, object]] = []
        for index in range(self.device_file_list.count()):
            item = self.device_file_list.item(index)
            if item.checkState() == Qt.CheckState.Checked:
                file_info = item.data(Qt.ItemDataRole.UserRole)
                if isinstance(file_info, dict):
                    selected.append(file_info)
        if not selected:
            return
        initial = str(self.settings.value("camera/export_dir", str(Path.home() / "Videos")))
        destination = QFileDialog.getExistingDirectory(
            self,
            self._tr("choose_export_folder"),
            initial,
        )
        if not destination:
            return
        self.settings.setValue("camera/export_dir", destination)
        self._mark_device_files_queued(selected)
        self._start_device_file_job("export", selected, destination)

    def _mark_device_files_queued(self, files: list[dict[str, object]]) -> None:
        for file_info in files:
            self._device_file_status(str(file_info["remote_path"]), "queued", "")

    def _device_file_status(self, remote_path: str, state: str, detail: str) -> None:
        if self.device_file_list is None:
            return
        for index in range(self.device_file_list.count()):
            item = self.device_file_list.item(index)
            file_info = item.data(Qt.ItemDataRole.UserRole)
            if not isinstance(file_info, dict) or str(file_info.get("remote_path")) != remote_path:
                continue
            item.setData(Qt.ItemDataRole.UserRole + 2, state)
            item.setData(Qt.ItemDataRole.UserRole + 3, detail)
            self._render_device_file_item(item)
            return

    def _render_device_file_item(self, item: QListWidgetItem) -> None:
        file_info = item.data(Qt.ItemDataRole.UserRole)
        if not isinstance(file_info, dict):
            return
        base_detail = str(item.data(Qt.ItemDataRole.UserRole + 1) or "")
        state = str(item.data(Qt.ItemDataRole.UserRole + 2) or "")
        detail = str(item.data(Qt.ItemDataRole.UserRole + 3) or "")
        if not state:
            item.setText(f"{file_info['name']}\n{base_detail}")
            return
        status = self._tr(f"status_{state}")
        status_detail = detail if state in {"failed", "copied", "completed"} else base_detail
        item.setText(f"[{status}] {file_info['name']}\n{status_detail or base_detail}")

    def _retranslate_device_file_items(self) -> None:
        if self.device_file_list is None:
            return
        for index in range(self.device_file_list.count()):
            self._render_device_file_item(self.device_file_list.item(index))

    def _device_workflow_progress(
        self,
        overall_percent: int,
        stage_percent: int,
        stage: str,
        name: str,
    ) -> None:
        if self.device_export_progress is not None:
            self.device_export_progress.setValue(overall_percent)
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(
                self._tr(
                    "workflow_line",
                    stage=self._tr(stage),
                    stage_percent=stage_percent,
                    name=name,
                    overall_percent=overall_percent,
                )
            )

    def _device_files_exported(self, result: dict[str, object]) -> None:
        paths = list(result.get("paths", []))
        destination = str(result.get("destination", ""))
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(
                self._tr("export_complete", count=len(paths), destination=destination)
            )
        if self.device_export_progress is not None:
            self.device_export_progress.setValue(100)
        QMessageBox.information(
            self,
            self._tr("stage_complete"),
            self._tr("export_complete_dialog", count=len(paths), destination=destination),
        )

    def _device_file_failed(self, message: str) -> None:
        if self.device_files_status_label is not None:
            self.device_files_status_label.setText(self._tr("export_failed", message=message))
        if self.device_export_progress is not None:
            self.device_export_progress.setFormat(self._tr("status_failed"))
        QMessageBox.critical(self, self._tr("status_failed"), message)

    def _device_file_thread_finished(self) -> None:
        self.device_file_thread = None
        self.device_file_worker = None
        self.device_file_action = ""
        if self.device_files_refresh_button is not None:
            self.device_files_refresh_button.setEnabled(bool(self.current_device_serial))
        if self.device_file_list is not None:
            self.device_file_list.setEnabled(True)
        self._update_device_file_selection()

    def _editor_page(self) -> QWidget:
        page = QFrame()
        page.setObjectName("studioPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(24, 24, 24, 24)
        title = QLabel("剪辑功能（开发中）")
        title.setObjectName("appTitle")
        description = QLabel("后续将加入素材导入、基础剪切、时间线、色彩调整和 VR180 元数据导出。")
        description.setObjectName("mutedText")
        layout.addWidget(title)
        layout.addWidget(description)
        placeholder = QFrame()
        placeholder.setObjectName("infoCard")
        placeholder_layout = QVBoxLayout(placeholder)
        placeholder_layout.setContentsMargins(24, 24, 24, 24)
        status = QLabel("开发中")
        status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status.setObjectName("appTitle")
        detail = QLabel("当前版本暂不提供剪辑处理，防抖和相机管理页面可正常独立使用。")
        detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        detail.setWordWrap(True)
        detail.setObjectName("mutedText")
        placeholder_layout.addStretch(1)
        placeholder_layout.addWidget(status)
        placeholder_layout.addWidget(detail)
        placeholder_layout.addStretch(1)
        layout.addWidget(placeholder, 1)
        return page

    def _apply_theme(self, theme: str, persist: bool = True) -> None:
        self.theme_name = "pixel" if theme == "pixel" else "minimal"
        for widget in self.findChildren(QWidget):
            if widget.styleSheet():
                widget.setStyleSheet("")
        self.setStyleSheet(PIXEL_THEME if self.theme_name == "pixel" else MINIMAL_THEME)
        self._refresh_brand_logo()
        if hasattr(self, "minimal_theme_action"):
            self.minimal_theme_action.setChecked(self.theme_name == "minimal")
            self.pixel_theme_action.setChecked(self.theme_name == "pixel")
        if persist:
            self.settings.setValue("ui/theme", self.theme_name)

    def _refresh_brand_logo(self) -> None:
        if self.brand_logo is None:
            return
        filename = (
            "slam-xcam-studio-horizontal-transparent-light.png"
            if self.theme_name == "pixel"
            else "slam-xcam-studio-horizontal-transparent.png"
        )
        pixmap = QPixmap(str(resource_path(f"assets/branding/{filename}")))
        self.brand_logo.setPixmap(
            pixmap.scaled(
                self.brand_logo.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def _tr(self, key: str, **values: object) -> str:
        return ui_text(key, self.language, **values)

    def _translate_literal(self, text: str) -> str:
        for entry in UI_TEXT.values():
            if text in entry.values():
                return entry[self.language]
        return text

    def _apply_language(self, language: str, persist: bool = True) -> None:
        self.language = "zh_CN" if language == "zh_CN" else "en"
        if persist:
            self.settings.setValue("ui/language", self.language)

        for widget_type in (QLabel, QPushButton, QToolButton, QGroupBox):
            for widget in self.findChildren(widget_type):
                current = widget.text() if hasattr(widget, "text") else widget.title()
                translated = self._translate_literal(current)
                if isinstance(widget, QGroupBox):
                    widget.setTitle(translated)
                else:
                    widget.setText(translated)
        for combo in self.findChildren(QComboBox):
            for index in range(combo.count()):
                combo.setItemText(index, self._translate_literal(combo.itemText(index)))

        if hasattr(self, "menu_button"):
            self.menu_button.setText(self._tr("menu"))
            self.ui_menu.setTitle(self._tr("ui_settings"))
            self.language_menu.setTitle(self._tr("language"))
            self.minimal_theme_action.setText(self._tr("theme_minimal"))
            self.pixel_theme_action.setText(self._tr("theme_pixel"))
            self.english_action.setText(self._tr("english"))
            self.chinese_action.setText(self._tr("chinese"))
            self.english_action.setChecked(self.language == "en")
            self.chinese_action.setChecked(self.language == "zh_CN")

        for button, key in zip(
            self.page_tab_buttons,
            ("page_stabilization", "page_camera", "page_editor"),
        ):
            button.setText(self._tr(key))
        for button, key in zip(
            self.left_tab_buttons,
            ("media_setting", "color_correction", "algorithm_strategy"),
        ):
            button.setText(self._tr(key))

        self.imu_algorithm_label.setText(self._tr("active_imu"))
        self.image_algorithm_label.setText(self._tr("active_image"))
        self.stabilization_mode_label.setText(self._tr("active_horizon"))
        self._refresh_algorithm_info()
        self._refresh_preview_details()
        self._retranslate_device_file_items()
        if persist:
            QTimer.singleShot(0, self.refresh_device_info)

    def _media_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("mediaPanel")
        panel.setFixedWidth(360)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(9)

        tab_bar = QFrame()
        tab_bar.setObjectName("segmentedTabs")
        tab_layout = QHBoxLayout(tab_bar)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(0)

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
        start.setMinimumHeight(40)
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
        tab.setObjectName("settingsPage")
        tab.setStyleSheet("background: #ffffff;")
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._field("SLAM XCAM Model", self.model))
        layout.addWidget(self._field("Video Mode", self.video_mode))
        layout.addWidget(self._field("畸变矫正", self.distortion_correction))
        layout.addWidget(self._field("视场角", self.field_of_view))
        layout.addWidget(
            self._field(
                "Choose Video",
                self._file_row(self.sbs, "Video files (*.mp4 *.mov *.mkv);;All files (*.*)", "Choose Video"),
            )
        )
        hint = QLabel("选择视频后优先匹配同目录的 video_motion.slamimu，也兼容旧版 video_imu.csv。")
        hint.setWordWrap(True)
        hint.setObjectName("mutedText")
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
        self.auto_match_label.setObjectName("matchBox")
        self.auto_match_label.hide()
        layout.addWidget(self.auto_match_label)
        layout.addStretch(1)
        return tab

    def _color_correction_tab(self) -> QWidget:
        tab = QWidget()
        tab.setObjectName("settingsPage")
        tab.setStyleSheet("background: #ffffff;")
        form = QVBoxLayout(tab)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(8)
        form.addWidget(self._field("LUT", self.lut))
        form.addWidget(self._field("Exposure", self.exposure))
        form.addWidget(self._field("Contrast", self.contrast))
        form.addWidget(self._field("Saturation", self.saturation))
        form.addWidget(self._field("White Balance", self.temperature))
        note = QLabel("LUT library: config/luts/library.json. LUT files will be added under config/luts/files/.")
        note.setWordWrap(True)
        note.setObjectName("mutedText")
        form.addWidget(note)
        form.addStretch(1)
        return tab

    def _algorithm_strategy_tab(self) -> QWidget:
        tab = QWidget()
        tab.setObjectName("settingsPage")
        tab.setStyleSheet("background: #ffffff;")
        form = QVBoxLayout(tab)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(8)
        self.imu_algorithm_info.setObjectName("infoBox")
        self.image_algorithm_info.setObjectName("infoBox")
        self.stabilization_mode_info.setObjectName("infoBox")
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
        label.setObjectName("appTitle")
        return label

    def _field(self, label_text: str, control: QWidget) -> QWidget:
        field = QWidget()
        field.setObjectName("fieldRow")
        field.setMinimumHeight(56)
        layout = QVBoxLayout(field)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        label = QLabel(label_text)
        label.setObjectName("fieldLabel")
        layout.addWidget(label)
        layout.addWidget(control)
        return field

    def _preview_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("previewPanel")
        panel.setMinimumWidth(360)
        panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        preview_head = QWidget()
        preview_head.setObjectName("previewHeader")
        head_layout = QVBoxLayout(preview_head)
        head_layout.setContentsMargins(14, 10, 14, 10)
        head_layout.setSpacing(5)
        title_row = QHBoxLayout()
        title_row.setContentsMargins(0, 0, 0, 0)
        title_row.setSpacing(12)
        self.preview_title.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.preview_title.setObjectName("previewTitle")
        self.preview_title.setMinimumWidth(0)
        self.preview_title.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.playback_button = QPushButton("Play")
        self.playback_button.setFixedWidth(76)
        self.playback_button.setMinimumHeight(32)
        self.playback_button.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.playback_button.setProperty("secondary", True)
        self.playback_button.clicked.connect(self.toggle_playback)
        self.preview_title.setMaximumWidth(320)
        title_row.addWidget(self.preview_title)
        title_row.addStretch(1)
        title_row.addWidget(self.playback_button, 0, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight)
        self.preview_stats.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.preview_stats.setWordWrap(True)
        self.preview_stats.setObjectName("mutedText")
        self.preview_stats.setMinimumWidth(0)
        self.preview_stats.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        head_layout.addLayout(title_row)
        head_layout.addWidget(self.preview_stats)
        layout.addWidget(preview_head)

        preview = QFrame()
        preview.setObjectName("previewViewport")
        preview.setMinimumWidth(0)
        preview.setMinimumHeight(220)
        preview_layout = QVBoxLayout(preview)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        self.video_widget.setMinimumHeight(180)
        self.video_widget.setMinimumWidth(0)
        self.video_widget.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.video_widget.setObjectName("videoSurface")
        self.video_widget.hide()
        self.preview_image.setMinimumHeight(180)
        self.preview_image.setMinimumWidth(0)
        self.preview_image.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.preview_image.setObjectName("previewSurface")
        preview_layout.addWidget(self.preview_image, 1)
        preview_layout.addWidget(self.video_widget, 1)
        layout.addWidget(preview, 1)

        job = QWidget()
        job_layout = QVBoxLayout(job)
        job_layout.setContentsMargins(14, 10, 14, 12)
        progress_row = QHBoxLayout()
        self.progress_text = QLabel("Ready")
        self.progress_percent = QLabel("0%")
        progress_row.addWidget(self.progress_text)
        progress_row.addStretch(1)
        progress_row.addWidget(self.progress_percent)
        job_layout.addLayout(progress_row)
        job_layout.addWidget(self.preview_timeline)
        self.time_meta = QLabel("Elapsed 00:00 · ETA --:--")
        self.time_meta.setObjectName("mutedText")
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
        row.setObjectName("controlRow")
        row.setMinimumWidth(0)
        row.setFixedHeight(36)
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        line.setMinimumWidth(0)
        line.setFixedHeight(36)
        button = QPushButton(button_text)
        button.setProperty("secondary", True)
        button.setFixedWidth(112)
        button.setFixedHeight(36)
        button.clicked.connect(lambda: self._pick_file(line, filter_text))
        layout.addWidget(line, 1)
        layout.addWidget(button)
        return row

    def _folder_row(self, line: QLineEdit) -> QWidget:
        row = QWidget()
        row.setObjectName("controlRow")
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        button = QPushButton("Browse")
        button.clicked.connect(lambda: self._pick_folder(line))
        layout.addWidget(line, 1)
        layout.addWidget(button)
        return row

    def _save_row(self, line: QLineEdit, filter_text: str) -> QWidget:
        row = QWidget()
        row.setObjectName("controlRow")
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
        self.imu_algorithm_info.setText(self._tr("imu_info"))
        self.image_algorithm_info.setText(self._tr("image_info"))
        self.stabilization_mode_info.setText(self._tr("horizon_info"))

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
        if video.stem.endswith("_8K50_SOURCE"):
            export_stem = video.stem.removesuffix("_SOURCE")
            default_output = save_dir / f"{export_stem}_VR180.mp4"
        else:
            default_output = save_dir / f"{video.stem}_stabilized_prototype.mp4"
        default_report = save_dir / f"{video.stem}_pair_report.json"
        if force or not self.output.text():
            self.output.setText(str(default_output))
        if force or not self.report.text():
            self.report.setText(str(default_report))
        self._refresh_preview_details()

    def _refresh_video_probe_summary(self, video: Path) -> None:
        try:
            source = inspect_slam_source(video)
            if source is not None:
                probe = ffprobe_json(video)
                stream = primary_video_stream(probe)
                duration = stream_duration_s(stream) or format_duration_s(probe)
                self.video_size_text = (
                    f"2 x {source.eye_width} x {source.eye_height} "
                    f"→ {source.output_width} x {source.output_height}"
                )
                self.video_duration_text = self._format_duration_label(duration)
                self._refresh_preview_details()
                return
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
        backend_key = {
            "auto": "backend_auto",
            "discrete_gpu": "backend_discrete",
            "integrated_gpu": "backend_integrated",
            "cpu": "backend_cpu",
        }.get(str(self.render_backend.currentData()), "backend_auto")
        self.preview_stats.setText(
            f"{self.model.currentText()}  /  {self.video_mode.currentText()}  /  "
            f"{self.field_of_view.currentText()}  /  {self._tr('distortion')} {self.distortion_correction.currentText()}\n"
            f"{self._tr('pipeline_summary')}  /  {self._tr(backend_key)}\n"
            f"{self._tr('duration')} {self.video_duration_text}  /  "
            f"{self._tr('resolution')} {self.video_size_text}\n"
            f"{self._tr('video_folder')}: {video_folder}\n"
            f"{self._tr('save_folder')}: {save_folder}"
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
            QMessageBox.warning(self, "Input validation failed", str(exc))
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
        source = None
        if mode == "sbs" and payload.get("sbs"):
            source = inspect_slam_source(Path(payload["sbs"]))
        if not payload.get("imu") and source is None:
            raise ValueError("Please select an IMU or SLAM motion file.")
        if action == "prototype" and not payload.get("output"):
            raise ValueError("Please choose an output MP4 path.")
        if (action == "prototype" and source is None
                and normalize_image_algorithm(payload.get("image_algorithm", "")) != "reference-renderer"):
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
        elif "Source analysis complete." in message:
            lines = [line for line in message.splitlines() if line.strip()]
            self.preview_title.setText("8K50 source analysis complete")
            self.preview_stats.setText("\n".join(lines[1:5]))
            QMessageBox.information(self, "Analysis complete", "8K50 source container validation is complete.")
        elif "8K50 source export complete." in message:
            self.preview_title.setText("8K50 VR180 export complete")
            self.preview_stats.setText("\n".join(message.splitlines()[1:5]))
            QMessageBox.information(self, "Export complete", "Standard 8K50 VR180 MP4 export is complete.")
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
