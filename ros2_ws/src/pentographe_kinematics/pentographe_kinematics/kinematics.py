"""Modèles géométriques direct et inverse du pantographe (système 5 barres).

Ce module ne dépend PAS de ROS : il ne contient que les mathématiques.
Il peut donc être testé seul, avec pytest, sans installer ROS.

Repère : A1 = (0, 0) est l'axe du moteur gauche, A5 = (l0, 0) l'axe du moteur
droit, x0 dirigé de A1 vers A5, y0 vers le haut. Les angles q1 et q4 sont
mesurés depuis x0, dans le sens trigonométrique. Toutes les longueurs sont en
millimètres, tous les angles en radians.
"""

import math
from dataclasses import dataclass


@dataclass
class PantographParams:
    """Dimensions du pantographe. Valeurs par défaut = valeurs mesurées sur le robot."""
    l0: float = 137.0   # entraxe des deux moteurs A1A5
    l1: float = 86.0    # bras gauche   A1A2
    l2: float = 150.0   # avant-bras gauche  A2A3
    l3: float = 150.0   # avant-bras droit   A3A4
    l4: float = 85.0    # bras droit    A4A5
    e_par: float = -16.0  # décalage du crayon le long de A2->A3, au-delà de A3
    e_perp: float = 26.0  # décalage du crayon perpendiculaire à A2->A3


class Unreachable(ValueError):
    """Levée quand le point demandé est hors de l'espace de travail."""


def _circle_intersection(P, r1, Q, r2, sign):
    """Point à la distance r1 de P et r2 de Q.

    sign = +1 ou -1 choisit l'une des deux solutions (configuration du bras).
    """
    dx, dy = Q[0] - P[0], Q[1] - P[1]
    d = math.hypot(dx, dy)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0.0:
        raise Unreachable("les deux cercles ne se coupent pas")
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r1 * r1 - a * a))
    ux, uy = dx / d, dy / d
    # normale (ux, uy) tournée de +90 degrés = (-uy, ux)
    return (P[0] + a * ux - sign * h * uy,
            P[1] + a * uy + sign * h * ux)


def forward(q1, q4, p=PantographParams()):
    """Modèle géométrique DIRECT : angles moteurs (q1, q4) -> position (x, y) du crayon."""
    A2 = (p.l1 * math.cos(q1), p.l1 * math.sin(q1))
    A4 = (p.l0 + p.l4 * math.cos(q4), p.l4 * math.sin(q4))
    A3 = _circle_intersection(A2, p.l2, A4, p.l3, +1)
    q2 = math.atan2(A3[1] - A2[1], A3[0] - A2[0])
    x = A3[0] + p.e_par * math.cos(q2) - p.e_perp * math.sin(q2)
    y = A3[1] + p.e_par * math.sin(q2) + p.e_perp * math.cos(q2)
    return (x, y)


def inverse(x, y, p=PantographParams()):
    """Modèle géométrique INVERSE : position (x, y) du crayon -> angles moteurs (q1, q4)."""
    E = (x, y)
    L = math.hypot(p.l2 + p.e_par, p.e_perp)
    delta = math.atan2(p.e_perp, p.l2 + p.e_par)
    # chaîne gauche : A2 à l1 de A1 et L de E
    A2 = _circle_intersection((0.0, 0.0), p.l1, E, L, +1)
    q1 = math.atan2(A2[1], A2[0])
    # position de A3
    q2 = math.atan2(E[1] - A2[1], E[0] - A2[0]) - delta
    A3 = (A2[0] + p.l2 * math.cos(q2), A2[1] + p.l2 * math.sin(q2))
    # chaîne droite : A4 à l3 de A3 et l4 de A5
    A4 = _circle_intersection(A3, p.l3, (p.l0, 0.0), p.l4, +1)
    q4 = math.atan2(A4[1], A4[0] - p.l0)
    return (q1, q4)


def all_joint_angles(x, y, p=PantographParams()):
    """Les quatre orientations absolues des barres, pour la simulation RViz.

    À partir de la position (x, y) du crayon, renvoie un dictionnaire des quatre
    angles ABSOLUS (orientation de chaque barre dans le repère mathématique XY,
    mesurée depuis x0), en radians :

      theta1 : orientation de link1  (A1 -> A2)   — bras gauche (moteur)
      theta2 : orientation de link2  (A2 -> A3)   — avant-bras gauche
      theta3 : orientation de link3  (A4 -> A3)   — avant-bras droit
      theta4 : orientation de link4  (A5 -> A4)   — bras droit (moteur)

    Le nœud de simulation convertit ensuite ces angles dans le repère de l'URDF.
    Lève Unreachable si le point est hors de l'espace de travail.
    """
    E = (x, y)
    L = math.hypot(p.l2 + p.e_par, p.e_perp)
    delta = math.atan2(p.e_perp, p.l2 + p.e_par)
    A2 = _circle_intersection((0.0, 0.0), p.l1, E, L, +1)
    theta1 = math.atan2(A2[1], A2[0])                       # link1 : A1 -> A2
    theta2 = math.atan2(E[1] - A2[1], E[0] - A2[0]) - delta  # link2 : A2 -> A3
    A3 = (A2[0] + p.l2 * math.cos(theta2), A2[1] + p.l2 * math.sin(theta2))
    A4 = _circle_intersection(A3, p.l3, (p.l0, 0.0), p.l4, +1)
    theta4 = math.atan2(A4[1], A4[0] - p.l0)                # link4 : A5 -> A4
    theta3 = math.atan2(A3[1] - A4[1], A3[0] - A4[0])       # link3 : A4 -> A3
    return {"theta1": theta1, "theta2": theta2,
            "theta3": theta3, "theta4": theta4}
