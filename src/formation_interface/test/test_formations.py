import math

from formation_interface.formations import FORMATIONS, compute_targets


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


def test_unknown_formation_raises():
    try:
        compute_targets("banana", 5, 1.0, (0.0, 0.0), 1.0)
    except ValueError:
        return
    assert False, "expected ValueError for unknown formation"
