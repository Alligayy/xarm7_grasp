import math
import pytest
from xarm7_grasp.tactile_force import ForceRule


def test_acquire_hold_and_unload():
    r = ForceRule()
    assert r.decide((0., 0.), 0., 800.) == (798., False)
    assert r.decide((1., 1.1), 0.2, 600.) == (600., True)
    assert r.decide((1.3, 0.9), 0.3, 600.) == (602., False)


def test_asymmetric_contact_does_not_crush_loaded_finger():
    assert ForceRule().decide((1., 0.), 0.3, 600.) == (600., False)


@pytest.mark.parametrize('forces,peak,pos', [((2., 1.), .3, 600.),
    ((1., 1.), .7, 600.), ((0., 0.), 0., 100.), ((math.nan, 0.), 0., 600.)])
def test_faults(forces, peak, pos):
    with pytest.raises(ValueError):
        ForceRule().decide(forces, peak, pos)


def test_invalid_parameters():
    with pytest.raises(ValueError):
        ForceRule(target=3.)
