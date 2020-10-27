#!/usr/bin/env python3

from amino import Vec3, XAngle, YAngle, ZAngle, AxAng, EulerRPY, Quat, RotMat, TfMat, DualQuat, QuatTrans
from math import pi


def h1(name):
    """Print a level 1 heading"""
    print("")
    print("{:^16}".format(name))
    print("{:=^16}".format(''))


def h2(name):
    """Print a level 2 heading"""
    print("")
    print("{:^16}".format(name))
    print("{:-^16}".format(''))


def check_equiv_tf(a, b):
    eps = 1e-6
    rel = (a * ~b)
    assert rel.translation.nrm2() < eps
    assert rel.rotation.ln().nrm2() < eps


def check_equiv_vec(a, b):
    eps = 1e-6
    assert a.ssd(b) < eps


h1("Construction")

# parent frame: 0
# child frame:  1
rot_0_1 = ZAngle(pi / 4)
trans_0_1 = Vec3([1, 2, 3])
tf_0_1 = (rot_0_1, trans_0_1)

h2("Dual Quaternion")
S_0_1 = DualQuat(tf_0_1)
print(S_0_1)

h2("Tf Matrix")
T_0_1 = TfMat(tf_0_1)
print(T_0_1)

h2("Quaternion-Translation")
E_0_1 = QuatTrans(tf_0_1)
print(E_0_1)

h1("Conversions")

h2("Tf Matrix")
T_S = TfMat(S_0_1)
T_E = TfMat(E_0_1)
check_equiv_tf(T_0_1, T_S)
check_equiv_tf(T_0_1, T_E)
print(T_S)
print(T_E)

h2("Dual Quaternion")
S_T = DualQuat(T_0_1)
S_E = DualQuat(E_0_1)
check_equiv_tf(S_0_1, S_T)
check_equiv_tf(S_0_1, S_E)
print(S_T)
print(S_E)

h2("Quaternion-Translation")
E_T = QuatTrans(T_0_1)
E_S = QuatTrans(S_0_1)
check_equiv_tf(E_0_1, E_S)
check_equiv_tf(E_0_1, E_T)
print(E_T)
print(E_S)

h1("Transform")

h2("Initial Point")
# point a in frame 1
p_1_a = Vec3([3, 5, 7])
print(p_1_a)

h2("Transformed Point")
# transform to frame 0
p_0_a_T = T_0_1.transform(p_1_a)
p_0_a_E = E_0_1.transform(p_1_a)
p_0_a_S = S_0_1.transform(p_1_a)

check_equiv_vec(p_0_a_T, p_0_a_E)
check_equiv_vec(p_0_a_S, p_0_a_E)

print(p_0_a_T)
print(p_0_a_E)
print(p_0_a_S)

h1("Inverse Transform")

Ti = ~T_0_1
Si = ~S_0_1
Ei = ~E_0_1

# inverse transform back to frame 1
# same as original p_1_a
p_1_a_T = Ti.transform(p_0_a_T)
p_1_a_S = Si.transform(p_0_a_S)
p_1_a_E = Ei.transform(p_0_a_S)

check_equiv_vec(p_1_a, p_1_a_T)
check_equiv_vec(p_1_a, p_1_a_S)
check_equiv_vec(p_1_a, p_1_a_E)

print(p_1_a_T)
print(p_1_a_S)
print(p_1_a_E)

h1("Chaining Transforms")

rot_1_2 = YAngle(pi / 2)
trans_1_2 = Vec3([2, 4, 8])
tf_1_2 = (rot_1_2, trans_1_2)

S_1_2 = DualQuat(tf_1_2)
T_1_2 = TfMat(tf_1_2)
E_1_2 = QuatTrans(tf_1_2)

h2("Initial Point")
# point b in frame 2

p_2_b = Vec3([2, 1, 0])
print(p_2_b)

h2("Successive Transforms")

# Transform twice
p_1_b_T = T_1_2.transform(p_2_b)
p_0_b_T = T_0_1.transform(p_1_b_T)

p_1_b_E = E_1_2.transform(p_2_b)
p_0_b_E = E_0_1.transform(p_1_b_E)

p_1_b_S = S_1_2.transform(p_2_b)
p_0_b_S = S_0_1.transform(p_1_b_S)

check_equiv_vec(p_1_b_T, p_1_b_E)
check_equiv_vec(p_1_b_S, p_1_b_E)

print(p_0_b_E)
print(p_0_b_T)
print(p_0_b_S)

h2("Chained Transforms")

S_0_2 = S_0_1 * S_1_2
T_0_2 = T_0_1 * T_1_2
E_0_2 = E_0_1 * E_1_2

print(S_0_2)
print(T_0_2)
print(E_0_2)

h2("Transformation via chained transform")

# Use chained transform
p_0_b_T_chain = T_0_2.transform(p_2_b)
p_0_b_E_chain = E_0_2.transform(p_2_b)
p_0_b_S_chain = S_0_2.transform(p_2_b)

check_equiv_vec(p_0_b_E_chain, p_0_b_T_chain)
check_equiv_vec(p_0_b_E_chain, p_0_b_S_chain)

check_equiv_vec(p_0_b_T_chain, p_0_b_T)
check_equiv_vec(p_0_b_E_chain, p_0_b_E)
check_equiv_vec(p_0_b_S_chain, p_0_b_S)

print(p_0_b_E_chain)
print(p_0_b_T_chain)
print(p_0_b_S_chain)
