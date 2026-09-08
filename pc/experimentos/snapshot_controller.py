import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PC_DIR = REPO_ROOT / "pc"
SNAPSHOT_PATH = (
    REPO_ROOT
    / "paper"
    / "reproducibilidad"
    / "software"
    / "pc"
    / "pc_servidor_vision.py"
)

for path in (Path(__file__).resolve().parent, PC_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

spec = importlib.util.spec_from_file_location("paper_experimental_controller", SNAPSHOT_PATH)
if spec is None or spec.loader is None:
    raise ImportError(f"No se pudo cargar la instantánea: {SNAPSHOT_PATH}")
controller = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = controller
spec.loader.exec_module(controller)

DISCOVERY_PORT = controller.DISCOVERY_PORT
MultiRobotApp = controller.MultiRobotApp
fsm_tangential_component = controller.fsm_tangential_component
merge_aruco_detections = controller.merge_aruco_detections
