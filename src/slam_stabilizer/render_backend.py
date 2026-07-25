from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import platform
import subprocess

from .process import hidden_subprocess_kwargs


@dataclass(frozen=True)
class RenderDevice:
    name: str
    vendor: str
    kind: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class RenderBackendSelection:
    preference: str
    selected: str
    selected_name: str
    available_devices: list[RenderDevice]
    note: str

    def to_dict(self) -> dict[str, object]:
        return {
            "preference": self.preference,
            "selected": self.selected,
            "selected_name": self.selected_name,
            "available_devices": [device.to_dict() for device in self.available_devices],
            "note": self.note,
        }


def detect_render_devices() -> list[RenderDevice]:
    if platform.system().lower() != "windows":
        return []
    # WMI is cleaner, but may be blocked in packaged/no-console runs. PnP is the fallback.
    devices = _detect_devices_wmi()
    if devices:
        return devices
    return _detect_devices_pnputil()


def _detect_devices_wmi() -> list[RenderDevice]:
    script = (
        "Get-CimInstance Win32_VideoController | "
        "Select-Object Name,AdapterCompatibility | ConvertTo-Json -Compress"
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
            **hidden_subprocess_kwargs(),
        )
        text = result.stdout.strip()
        if not text:
            return []
        raw = json.loads(text)
        entries = raw if isinstance(raw, list) else [raw]
    except Exception:
        return []

    devices: list[RenderDevice] = []
    for entry in entries:
        name = str(entry.get("Name") or "").strip()
        vendor = str(entry.get("AdapterCompatibility") or "").strip()
        if not name:
            continue
        devices.append(RenderDevice(name=name, vendor=vendor, kind=_classify_device(name, vendor)))
    return devices


def _detect_devices_pnputil() -> list[RenderDevice]:
    try:
        result = subprocess.run(
            ["pnputil", "/enum-devices", "/class", "Display"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
            **hidden_subprocess_kwargs(),
        )
    except Exception:
        return []

    devices: list[RenderDevice] = []
    current: dict[str, str] = {}
    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            _append_pnputil_device(devices, current)
            current = {}
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        current[key.strip().lower()] = value.strip()
    _append_pnputil_device(devices, current)
    return devices


def _append_pnputil_device(devices: list[RenderDevice], entry: dict[str, str]) -> None:
    name = entry.get("device description", "") or entry.get("设备描述", "")
    vendor = entry.get("manufacturer name", "") or entry.get("制造商名称", "")
    status = entry.get("status", "") or entry.get("状态", "")
    status_l = status.lower()
    if not name or status_l == "disabled" or "禁用" in status:
        return
    kind = _classify_device(name, vendor)
    if kind == "virtual_display":
        return
    devices.append(RenderDevice(name=name, vendor=vendor, kind=kind))


def select_render_backend(preference: str = "auto") -> RenderBackendSelection:
    normalized = (preference or "auto").strip().lower()
    devices = detect_render_devices()
    if normalized == "cpu":
        return RenderBackendSelection(normalized, "cpu", "CPU", devices, "User selected CPU rendering.")

    if normalized in {"discrete_gpu", "integrated_gpu"}:
        match = _first_device(devices, normalized)
        if match:
            return RenderBackendSelection(normalized, normalized, match.name, devices, f"User selected {normalized}.")
        return RenderBackendSelection(normalized, "cpu", "CPU", devices, f"Requested {normalized} was not detected; falling back to CPU.")

    discrete = _first_device(devices, "discrete_gpu")
    if discrete:
        return RenderBackendSelection("auto", "discrete_gpu", discrete.name, devices, "Auto selected discrete GPU first.")
    integrated = _first_device(devices, "integrated_gpu")
    if integrated:
        return RenderBackendSelection("auto", "integrated_gpu", integrated.name, devices, "Auto selected integrated GPU after no discrete GPU was detected.")
    return RenderBackendSelection("auto", "cpu", "CPU", devices, "No GPU was detected; falling back to CPU.")


def _first_device(devices: list[RenderDevice], kind: str) -> RenderDevice | None:
    return next((device for device in devices if device.kind == kind), None)


def _classify_device(name: str, vendor: str) -> str:
    text = f"{name} {vendor}".lower()
    # Ignore remote-display and virtual-display adapters; they cannot accelerate rendering.
    if any(token in text for token in ("virtual display", "idd", "oray", "asklink")):
        return "virtual_display"
    if any(token in text for token in ("780m", "760m", "740m", "680m", "660m", "vega", "radeon graphics")):
        return "integrated_gpu"
    if any(token in text for token in ("nvidia", "geforce", "rtx", "gtx", "quadro", "radeon rx", "radeon pro")):
        return "discrete_gpu"
    if any(token in text for token in ("intel", "iris", "uhd graphics", "arc graphics")):
        return "integrated_gpu"
    if any(token in text for token in ("amd", "radeon")):
        return "gpu"
    return "gpu"
