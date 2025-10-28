import numpy as np
from scipy.linalg import solve_continuous_are
from dynamics import q_err

def detumble_controller(t, q, w, q_ref, w_ref, params):
    Kd = np.array([0.08,0.08,0.06]); return -Kd*w

def pid_att_controller(t, q, w, q_ref, w_ref, params):
    qe = q_err(q_ref, q); 
    if qe[0] < 0: qe = -qe
    e = qe[1:]; ew = w - w_ref
    Kp=np.array([0.35,0.35,0.28]); Kd=np.array([0.10,0.10,0.08])
    return -Kp*e - Kd*ew

def lqr_gain(J, q_cost=10.0, w_cost=1.0, u_cost=0.5):
    A = np.block([[np.zeros((3,3)), np.eye(3)], [np.zeros((3,3)), np.zeros((3,3))]])
    B = np.block([[np.zeros((3,3))],[np.linalg.inv(J)]])
    Q = np.block([[q_cost*np.eye(3), np.zeros((3,3))],[np.zeros((3,3)), w_cost*np.eye(3)]])
    R = u_cost*np.eye(3)
    P = solve_continuous_are(A,B,Q,R)
    return np.linalg.inv(R) @ (B.T @ P)

_cache = {}
def lqr_att_controller(t, q, w, q_ref, w_ref, params):
    key = tuple(params.J.flatten())
    if key not in _cache: _cache[key] = lqr_gain(params.J)
    K = _cache[key]
    qe = q_err(q_ref, q); 
    if qe[0] < 0: qe = -qe
    x = np.hstack([qe[1:], w-w_ref])
    return -K @ x
