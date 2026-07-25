from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from slam_stabilizer.qt_gui import main


raise SystemExit(main())

