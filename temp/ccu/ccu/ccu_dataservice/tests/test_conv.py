# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from inspect import currentframe as cf

from ccu_bautiro.NodeTree import CDrillHole, CDrillMask, CFastener, CNode
from ccu_bautiro.Types import CMatrix
from ccu_dataservice.conv import rel2abs, tm2orient, tm2position


def __begin(cf):
    print('### RUNNING: ' + cf.f_code.co_name.upper())


def __end(cf):
    print('### SUCCESS: ' + cf.f_code.co_name.upper())


def test_tm2pos():
    __begin(cf())
    tm: CMatrix = CMatrix()
    tm.v0x = 1.1
    tm.v0y = 22.22
    tm.v0z = 333.333
    result = tm2position(tm)

    assert 'tuple' == type(result).__name__
    assert 3 == len(result)
    assert 1.1 == result[0]
    assert 22.22 == result[1]
    assert 333.333 == result[2]
    __end(cf())


def test_tm2orient():
    __begin(cf())
    tm: CMatrix = CMatrix()
    tm.v0x = 1.1
    tm.v0y = 22.22
    tm.v0z = 333.333

    tm.v1x = 1.0
    tm.v1y = 0.0
    tm.v1z = 0.0

    tm.v2x = 0.0
    tm.v2y = 1.0
    tm.v2z = 0.0

    tm.v3x = 0.0
    tm.v3y = 0.0
    tm.v3z = 1.0

    result = tm2orient(tm)

    assert 'tuple' == type(result).__name__
    assert 4 == len(result)
    assert 0.0 == result[0]
    assert 0.0 == result[1]
    assert 0.0 == result[2]
    assert 1.0 == result[3]
    __end(cf())


def test_rel2abs():
    __begin(cf())
    root = CNode()
    root.tm = trans_x1()

    node1 = CNode(parent=root)
    node1.tm = trans_x1()
    node1.tm.v0y = 0.5

    node2 = CNode(parent=node1)
    node2.tm = trans_x1()
    node2.tm.v0y = 0.3

    fast = CFastener(parent=node2)

    dm = CDrillMask(parent=fast)
    dm.tm = trans_x1()

    dh1 = CDrillHole(parent=dm)
    dh1.tm = trans_x1()

    dh2 = CDrillHole(parent=dm)
    dh2.tm = trans_x1()
    dh2.tm.v0x = 2.6

    assert 1.0 == rel2abs(root).v0x
    assert 2.0 == rel2abs(node1).v0x
    assert 3.0 == rel2abs(node2).v0x
    assert 3.0 == rel2abs(fast).v0x
    assert 4.0 == rel2abs(dm).v0x

    assert 5.0 == rel2abs(dh1).v0x
    assert 6.6 == rel2abs(dh2).v0x
    __end(cf())


def trans_x1(): return CMatrix(v0x=1.0, v0y=0.0, v0z=0.0,
                               v1x=1.0, v1y=0.0, v1z=0.0,
                               v2x=0.0, v2y=1.0, v2z=0.0,
                               v3x=0.0, v3y=0.0, v3z=1.0)


def __run_all_tests():
    test_tm2pos()
    test_tm2orient()
    test_rel2abs()


if __name__ == '__main__':
    __run_all_tests()
