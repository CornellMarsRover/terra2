"""ROS-independent contract between mission selection and target perception."""


def encode_target_request(mission_index: int, target_name: str) -> str:
    """Encode the mission step separately from the requested visual target."""
    mission_index = int(mission_index)
    target_name = str(target_name).strip()
    if mission_index < 1 or not target_name or ":" in target_name:
        raise ValueError("invalid target request")
    return f"{mission_index}:{target_name}"


def decode_target_request(value: str):
    """Return mission index, target name, and optional ArUco marker ID."""
    try:
        mission_text, target_name = str(value).split(":", 1)
        mission_index = int(mission_text)
    except (TypeError, ValueError):
        raise ValueError("target request must be '<mission>:<name>'") from None
    if mission_index < 1 or not target_name or ":" in target_name:
        raise ValueError("invalid target request")
    marker_id = None
    if target_name.startswith("ar"):
        try:
            marker_id = int(target_name[2:])
        except ValueError:
            raise ValueError("ArUco target must end in a numeric ID") from None
        if marker_id < 0:
            raise ValueError("ArUco marker ID must not be negative")
    return mission_index, target_name, marker_id
