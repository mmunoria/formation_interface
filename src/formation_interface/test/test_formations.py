import math

from formation_interface.formations import (
    FORMATIONS,
    compute_targets,
    transform_offsets,
)


def test_exactly_three_builtin_formations():
    assert set(FORMATIONS) == {"line", "v", "circle"}


def test_every_formation_returns_n_targets():
    for name in FORMATIONS:
        targets = compute_targets(name, 5, 1.5, (0.0, 0.0), 2.0)
        assert len(targets) == 5
        for (x, y, z) in targets:
            assert not math.isnan(x)
            assert not math.isnan(y)
            assert z == 2.0


def test_line_is_centred_on_the_origin():
    targets = compute_targets("line", 5, 1.0, (0.0, 0.0), 1.0)
    cx = sum(p[0] for p in targets) / len(targets)
    cy = sum(p[1] for p in targets) / len(targets)
    assert abs(cx) < 1e-6
    assert abs(cy) < 1e-6


def test_center_offset_is_applied():
    targets = compute_targets("line", 3, 1.0, (10.0, -4.0), 1.5)
    cx = sum(p[0] for p in targets) / len(targets)
    cy = sum(p[1] for p in targets) / len(targets)
    assert abs(cx - 10.0) < 1e-6
    assert abs(cy + 4.0) < 1e-6


def test_custom_offsets_rotation_and_altitude():
    # One drone 1 m ahead (+X), yaw 90 deg -> ends up 1 m along +Y.
    targets = transform_offsets([(1.0, 0.0)], (0.0, 0.0), 2.0, yaw=math.pi / 2)
    x, y, z = targets[0]
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9
    assert z == 2.0


def test_custom_offsets_support_z():
    # dz stacks on top of the base altitude -> 3-D formations.
    targets = transform_offsets([(0.0, 0.0, 0.5)], (1.0, 1.0), 2.0)
    assert targets[0] == (1.0, 1.0, 2.5)


def test_unknown_formation_raises():
    try:
        compute_targets("banana", 5, 1.0, (0.0, 0.0), 1.0)
    except ValueError:
        return
    assert False, "expected ValueError for unknown formation"
