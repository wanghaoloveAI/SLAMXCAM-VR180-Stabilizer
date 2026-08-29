# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['SLAM_XCAM_Studio.pyw'],
    pathex=['src'],
    binaries=[
        ('calibration_runtime/slam_xcam_calibration_runtime.dll', 'calibration_runtime'),
    ],
    datas=[
        ('config', 'config'),
        ('assets/branding', 'assets/branding'),
        ('calibration_runtime/slam_xcam_calibration_runtime.manifest.json', 'calibration_runtime'),
        ('THIRD_PARTY_NOTICES.md', '.'),
        ('OFFICIAL_RUNTIME_LICENSE_NOTICE.md', '.'),
    ],
    hiddenimports=[
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
        'PySide6.QtMultimedia',
        'PySide6.QtMultimediaWidgets',
        'PySide6.QtOpenGL',
        'openpyxl',
        'sqlite3',
        'slam_stabilizer.vendor.pyvqf',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

# Workspace document tools add their private libheif/Poppler runtimes to PATH.
# They must not shadow Windows UCRT or Qt dependencies inside the packaged app.
a.binaries = [
    entry for entry in a.binaries
    if 'codex-runtimes' not in str(entry[1]).lower()
]
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='SLAM_XCAM_Studio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    icon=['assets/branding/slam-xcam-studio.ico'],
    entitlements_file=None,
)
