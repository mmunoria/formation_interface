# Drone profiles

Each `*.yaml` file here is one physical drone's parameters that are
drone-specific but NOT mission-specific: its PX4 uXRCE-DDS namespace,
MAVLink system id, OptiTrack pose topic, capability tags, speed/accel
limits, default deploy backend, and (for the `ssh` backend)
companion-computer connection details. See
`formation_interface/drone_profiles.py` for the loader/validator.

The `name:` field inside the file (not the filename) is the identity used
everywhere else: the mission GUI's assignment table,
`DroneTelemetry.drone_name`, and `missions/<name>/mission.yaml`'s
`assignment:` keys.

This is separate from the top-level `config/drones.yaml` used by
`formation_node`/`gui_node` (the formation-flying stack) - the two are
**not** kept in sync automatically. If a drone's namespace, system_id, or
pose topic changes, update both files.

Add a new drone by writing another `<name>.yaml` here - no code changes
needed; the mission GUI's "reload profiles" button picks it up.
