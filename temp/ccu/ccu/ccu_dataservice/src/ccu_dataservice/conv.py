# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from math import sqrt
from typing import List, Tuple

from ccu_bautiro import ITM, CMatrix
from numpy import eye, matmul
from ccu_dataservice.eutil import resolve


def matrix_to_quaternion(m11: float, m12: float, m13: float,
                         m21: float, m22: float, m23: float,
                         m31: float, m32: float, m33: float) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix intro quaternion."""
    t = m11 + m22 + m33
    if t > 0:
        w = sqrt(1.0 + t) / 2.0
        x = (m32 - m23) / (4.0 * w)
        y = (m13 - m31) / (4.0 * w)
        z = (m21 - m12) / (4.0 * w)
    else:
        if m11 > m22 and m11 > m33:
            s = 2.0 * sqrt(1.0 + m11 - m22 - m33)
            w = (m32 - m23) / s
            x = 0.25 * s
            y = (m12 + m21) / s
            z = (m13 + m31) / s
        elif m22 > m33:
            s = 2.0 * sqrt(1.0 + m22 - m11 - m33)
            w = (m13 - m31) / s
            x = (m12 + m21) / s
            y = 0.25 * s
            z = (m23 + m32) / s
        else:
            s = 2.0 * sqrt(1.0 + m33 - m11 - m22)
            w = (m21 - m12) / s
            x = (m13 + m31) / s
            y = (m23 + m32) / s
            z = 0.25 * s
    return x, y, z, w


def tm2orient(tm: CMatrix) -> Tuple[float, float, float, float]:
    return matrix_to_quaternion(tm.v1x, tm.v1y, tm.v1z,
                                tm.v2x, tm.v2y, tm.v2z,
                                tm.v3x, tm.v3y, tm.v3z)


def tm2position(tm: CMatrix) -> Tuple[float, float, float]:
    return tm.v0x, tm.v0y, tm.v0z


def tm2pose(tm: CMatrix) -> Tuple[float, float, float, float, float, float, float]:
    return tm2position(tm), tm2orient(tm)


# CDrillHole.tm
# CDrillHole.parent -> CDrillMask.tm
#                      CDrillMask.parent -> CFastener.parent -> CNode.tm
#                                                               CNode.parent -> CNode | None
def rel2abs(node_tree_element: ITM) -> CMatrix:
    """Return the entry elements `tm` multiplied with all parents `tm` (up to root-node)."""
    # From all Parents we collect their matrix (if they have one)
    matrices = []
    n = resolve(node_tree_element)
    while n.parent is not None:
        if hasattr(n, 'tm'):  # Fastener do not have 'tm'
            matrices.append(_to_4x4(n.tm))
        n = n.parent
    matrices.append(_to_4x4(n.tm))

    sum = eye(4)
    if 'lefty':
        for m in matrices:
            sum = matmul(sum, m)
    else:
        for m in reversed(matrices):
            sum = matmul(m, sum)

    return CMatrix(v1x=sum[0][0], v1y=sum[1][0], v1z=sum[2][0],
                   v2x=sum[0][1], v2y=sum[1][1], v2z=sum[2][1],
                   v3x=sum[0][2], v3y=sum[1][2], v3z=sum[2][2],
                   v0x=sum[0][3], v0y=sum[1][3], v0z=sum[2][3])


def _to_4x4(tm: CMatrix) -> List[List[float]]:
    m: List[List[float]]
    m = [[tm.v1x, tm.v2x, tm.v3x, tm.v0x],
         [tm.v1y, tm.v2y, tm.v3y, tm.v0y],
         [tm.v1z, tm.v2z, tm.v3z, tm.v0z],
         [0,           0,      0,      1]]
    return m
