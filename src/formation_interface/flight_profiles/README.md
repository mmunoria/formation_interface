# Flight profiles

Each `*.yaml` file here is one flight profile: a drone-agnostic description
of *how* to fly (target PX4 state, control scheme, shared params, and a
time-parametric reference trajectory) - see
`formation_interface/flight_profiles.py` for the loader/validator and
`sample_trajectory()`'s supported trajectory `type`s
(`static_point` | `circle` | `waypoints`).

Nothing here names a specific drone - that's `config/drone_profiles/`.
Assigning a flight profile to a drone happens per-mission, in the mission
GUI (or by hand-editing `missions/<name>/mission.yaml`'s `assignment:` map).

Add a new one by writing another `<name>.yaml` here (the `name:` field
inside the file is the identity used everywhere else, not the filename) -
no code changes needed; the mission GUI's "reload profiles" button picks it
up.
