import importlib.util
from pathlib import Path

from mavros_msgs.msg import ExtendedState

THIS_FILE = Path(__file__).resolve()
PACKAGE_ROOT = THIS_FILE.parents[1]
REPO_ROOT = THIS_FILE.parents[3]

MODULE_PATH = PACKAGE_ROOT / "drone_utils" / "simple_takeoff_service.py"
MODULE_SPEC = importlib.util.spec_from_file_location(
    "stream_contract_simple_takeoff_service",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(MODULE_SPEC)
assert MODULE_SPEC is not None and MODULE_SPEC.loader is not None
MODULE_SPEC.loader.exec_module(MODULE)
TOPIC_TYPE_MAP = MODULE.TOPIC_TYPE_MAP


def test_extended_state_supported_in_required_topic_map():
    assert TOPIC_TYPE_MAP["/mavros/extended_state"] is ExtendedState


def test_stream_rate_script_matches_simple_takeoff_contract():
    script = (REPO_ROOT / "set_stream_rate.sh").read_text(encoding="utf-8")
    assert "/mavros/set_stream_rate" in script
    assert "mavros_msgs/srv/StreamRate" in script
    assert "stream_id: 0" in script
    assert "message_rate: ${rate}" in script
    assert "on_off: true" in script
    assert "/mavros/set_message_interval" in script
    assert "mavros_msgs/srv/MessageInterval" in script
    assert "message_id: 245" in script
    assert "message_rate: ${extended_state_rate}" in script


def test_stream_check_script_matches_readiness_contract():
    script = (REPO_ROOT / "check_mavros_streams.sh").read_text(encoding="utf-8")
    assert "./set_stream_rate.sh" not in script
    assert '"/mavros/imu/data"' in script
    assert '"/mavros/local_position/pose"' in script
    assert '"/mavros/extended_state"' in script
    assert 'landed_state=0' in script
    assert 'ros2 topic echo --once "${topic}"' in script


def test_preflight_checks_config_sync_and_delegates_stream_check():
    script = (REPO_ROOT / "preflight_checks.sh").read_text(encoding="utf-8")
    assert "src/drone_mission_demo/config" in script
    assert "install/drone_mission_demo/share/drone_mission_demo/config" in script
    assert "src/drone_target_cv/config" in script
    assert "install/drone_target_cv/share/drone_target_cv/config" in script
    assert 'check_mavros_streams.sh" "${stream_rate}" "${extended_state_rate}" "${topic_timeout_s}"' in script


def test_demo_params_require_extended_state():
    for path in (
        REPO_ROOT / "src/drone_mission_demo/config/params/mission_params.yaml",
        REPO_ROOT / "src/drone_mission_demo/config/params/sim_target_mission.yaml",
    ):
        text = path.read_text(encoding="utf-8")
        assert '"/mavros/extended_state"' in text
