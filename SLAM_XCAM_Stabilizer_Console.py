from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from slam_stabilizer.inspect_pair import main as inspect_pair_main
from slam_stabilizer.models import VideoInput
from slam_stabilizer.pipeline import StabilizationJob, run_job


def resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", ROOT))
    return base / relative


def app_path(relative: str) -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / relative
    return ROOT / relative


def ask(label: str, default: Path | None = None) -> Path:
    suffix = f" [{default}]" if default else ""
    value = input(f"{label}{suffix}: ").strip().strip('"')
    return Path(value) if value else Path(default or "")


def inspect_pair() -> None:
    video = ask("SBS video path")
    imu = ask("IMU or SLAM motion path")
    report = ask("Report json output", app_path("outputs/pair_report.json"))
    original_argv = sys.argv[:]
    try:
        sys.argv = ["inspect_pair", "--video", str(video), "--imu", str(imu), "--json", str(report)]
        inspect_pair_main()
    finally:
        sys.argv = original_argv


def run_prototype() -> None:
    video = ask("SBS video path")
    imu = ask("IMU or SLAM motion path")
    lens = ask("Lens profile", resource_path("config/lenses/slam_xcam_2026.json"))
    calibration_raw = input("Calibration json path [optional]: ").strip().strip('"')
    output = ask("Output mp4", app_path("outputs/stabilizer_prototype.mp4"))

    plan = run_job(
        StabilizationJob(
            video=VideoInput(mode="sbs", input_sbs=video, input_left=None, input_right=None),
            imu_path=imu,
            lens_profile_path=lens,
            calibration_path=Path(calibration_raw) if calibration_raw else None,
            output_path=output,
        )
    )
    print(f"\nDone. Prototype video: {output}")
    print(f"Plan JSON: {plan}")


def main() -> int:
    print("SLAM XCAM VR180 Stabilizer Prototype")
    print("This is not the final stabilized renderer yet.")
    print()
    while True:
        print("1. Inspect video + IMU pair")
        print("2. Run prototype remux + VR180 metadata")
        print("3. Exit")
        choice = input("Choose: ").strip()
        try:
            if choice == "1":
                inspect_pair()
            elif choice == "2":
                run_prototype()
            elif choice == "3":
                return 0
            else:
                print("Unknown choice.")
        except Exception as exc:
            print(f"\nError: {exc}")
        print()
        input("Press Enter to continue...")
        print()


if __name__ == "__main__":
    raise SystemExit(main())

