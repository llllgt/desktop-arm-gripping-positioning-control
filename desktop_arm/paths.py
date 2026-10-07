from pathlib import Path
import os


def project_root() -> Path:
    """Editable installation by default; ROS installs pass the asset share directory."""
    return Path(os.environ.get('DESKTOP_ARM_ROOT', Path(__file__).resolve().parents[1]))
