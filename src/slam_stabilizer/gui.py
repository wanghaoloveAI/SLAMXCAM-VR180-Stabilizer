from __future__ import annotations

from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .models import VideoInput
from .pipeline import StabilizationJob, run_job


def _resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2]))
    return base / relative


def _working_path(relative: str) -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / relative
    return Path(relative)


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("SLAM XCAM VR180 Stabilizer Prototype")
        self.geometry("780x520")
        self.resizable(True, True)
        self._messages: queue.Queue[str] = queue.Queue()

        self.mode = tk.StringVar(value="sbs")
        self.input_sbs = tk.StringVar()
        self.input_left = tk.StringVar()
        self.input_right = tk.StringVar()
        self.imu = tk.StringVar()
        self.lens = tk.StringVar(value=str(_resource_path("config/lenses/slam_xcam_2026.json")))
        self.calibration = tk.StringVar()
        self.output = tk.StringVar(value=str(_working_path("outputs/stabilized_vr180.mp4")))

        self._build()
        self.after(100, self._poll_messages)

    def _build(self) -> None:
        root = ttk.Frame(self, padding=16)
        root.pack(fill=tk.BOTH, expand=True)

        ttk.Label(root, text="Input Mode").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(root, text="Stitched SBS 2:1", variable=self.mode, value="sbs").grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(root, text="Separate Left/Right 1:1", variable=self.mode, value="dual").grid(row=0, column=2, sticky="w")

        ttk.Label(
            root,
            text="Prototype status: opens on Windows, validates video/IMU, writes a plan, and remuxes VR180 metadata. Stabilized reprojection is still in development.",
            foreground="#8a4b00",
            wraplength=720,
        ).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 10))

        self._file_row(root, 2, "SBS Video", self.input_sbs, [("Video", "*.mp4 *.mov *.mkv"), ("All", "*.*")])
        self._file_row(root, 3, "Left Video", self.input_left, [("Video", "*.mp4 *.mov *.mkv"), ("All", "*.*")])
        self._file_row(root, 4, "Right Video", self.input_right, [("Video", "*.mp4 *.mov *.mkv"), ("All", "*.*")])
        self._file_row(root, 5, "IMU CSV", self.imu, [("CSV", "*.csv"), ("All", "*.*")])
        self._file_row(root, 6, "Lens Profile", self.lens, [("JSON", "*.json"), ("All", "*.*")])
        self._file_row(root, 7, "Calibration", self.calibration, [("JSON", "*.json"), ("All", "*.*")])
        self._save_row(root, 8, "Output Video", self.output)

        run = ttk.Button(root, text="Run Prototype", command=self._run)
        run.grid(row=9, column=0, columnspan=3, sticky="ew", pady=(12, 8))

        self.log = tk.Text(root, height=12, wrap="word")
        self.log.grid(row=10, column=0, columnspan=3, sticky="nsew")
        root.columnconfigure(1, weight=1)
        root.rowconfigure(10, weight=1)

    def _file_row(self, root: ttk.Frame, row: int, label: str, var: tk.StringVar, filetypes: list[tuple[str, str]]) -> None:
        ttk.Label(root, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(root, textvariable=var).grid(row=row, column=1, sticky="ew", pady=4)
        ttk.Button(root, text="Browse", command=lambda: self._pick_file(var, filetypes)).grid(row=row, column=2, sticky="ew", padx=(8, 0), pady=4)

    def _save_row(self, root: ttk.Frame, row: int, label: str, var: tk.StringVar) -> None:
        ttk.Label(root, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(root, textvariable=var).grid(row=row, column=1, sticky="ew", pady=4)
        ttk.Button(root, text="Save As", command=lambda: self._save_file(var)).grid(row=row, column=2, sticky="ew", padx=(8, 0), pady=4)

    def _pick_file(self, var: tk.StringVar, filetypes: list[tuple[str, str]]) -> None:
        path = filedialog.askopenfilename(filetypes=filetypes)
        if path:
            var.set(path)

    def _save_file(self, var: tk.StringVar) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".mp4", filetypes=[("MP4", "*.mp4"), ("All", "*.*")])
        if path:
            var.set(path)

    def _run(self) -> None:
        try:
            video = VideoInput(
                mode=self.mode.get(),
                input_sbs=Path(self.input_sbs.get()) if self.input_sbs.get() else None,
                input_left=Path(self.input_left.get()) if self.input_left.get() else None,
                input_right=Path(self.input_right.get()) if self.input_right.get() else None,
            )
            job = StabilizationJob(
                video=video,
                imu_path=Path(self.imu.get()),
                lens_profile_path=Path(self.lens.get()),
                calibration_path=Path(self.calibration.get()) if self.calibration.get() else None,
                output_path=Path(self.output.get()),
            )
        except Exception as exc:
            messagebox.showerror("Invalid job", str(exc))
            return

        self._write("Starting prototype pipeline...\n")
        threading.Thread(target=self._run_background, args=(job,), daemon=True).start()

    def _run_background(self, job: StabilizationJob) -> None:
        try:
            plan = run_job(job)
            self._messages.put(f"Done. Stabilization plan written to {plan}\n")
        except Exception as exc:
            self._messages.put(f"Error: {exc}\n")

    def _poll_messages(self) -> None:
        while not self._messages.empty():
            self._write(self._messages.get_nowait())
        self.after(100, self._poll_messages)

    def _write(self, text: str) -> None:
        self.log.insert(tk.END, text)
        self.log.see(tk.END)


def main() -> None:
    App().mainloop()


if __name__ == "__main__":
    main()
