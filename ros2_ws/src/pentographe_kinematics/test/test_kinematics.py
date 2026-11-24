"""Tests du module kinematics (lancés avec pytest, sans ROS)."""

import math
import pytest

from pentographe_kinematics import kinematics as k


def test_forward_renvoie_un_point():
    x, y = k.forward(2.0, 1.0)
    assert isinstance(x, float) and isinstance(y, float)


@pytest.mark.parametrize("q1, q4", [
    (2.0, 1.0),
    (1.8, 1.3),
    (2.3, 0.7),
    (1.5, 1.5),
])
def test_inverse_annule_forward(q1, q4):
    """MGI(MGD(q)) doit redonner les angles de départ."""
    x, y = k.forward(q1, q4)
    r1, r4 = k.inverse(x, y)
    assert r1 == pytest.approx(q1, abs=1e-9)
    assert r4 == pytest.approx(q4, abs=1e-9)


def test_point_hors_zone_leve_une_erreur():
    """Un point très loin n'est pas atteignable."""
    with pytest.raises(k.Unreachable):
        k.inverse(5000.0, 5000.0)


def test_parametres_personnalises():
    """On peut donner d'autres dimensions (ex. valeurs du prof)."""
    prof = k.PantographParams(l0=137.98, l1=80, l2=157, l3=147, l4=79,
                              e_par=0.0, e_perp=0.0)
    x, y = k.forward(2.0, 1.0, prof)
    r1, r4 = k.inverse(x, y, prof)
    assert r1 == pytest.approx(2.0, abs=1e-9)
    assert r4 == pytest.approx(1.0, abs=1e-9)



@pytest.mark.parametrize("q1, q4", [(2.0, 1.0), (1.8, 1.3), (2.3, 0.7)])
def test_all_joint_angles_coherent_avec_inverse(q1, q4):
    """Les orientations theta1 et theta4 (moteurs) renvoyées par all_joint_angles
    doivent coïncider avec les angles du modèle inverse."""
    x, y = k.forward(q1, q4)
    a = k.all_joint_angles(x, y)
    r1, r4 = k.inverse(x, y)
    assert a["theta1"] == pytest.approx(r1, abs=1e-9)
    assert a["theta4"] == pytest.approx(r4, abs=1e-9)
    assert set(a.keys()) == {"theta1", "theta2", "theta3", "theta4"}
