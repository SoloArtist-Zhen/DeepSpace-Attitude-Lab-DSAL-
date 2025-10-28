from __future__ import annotations
import numpy as np
from numpy.typing import NDArray

def q_norm(q):
    return q/np.linalg.norm(q)

def q_mul(q,p):
    w1,x1,y1,z1=q; w2,x2,y2,z2=p
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])

def q_conj(q):
    w,x,y,z=q; return np.array([w,-x,-y,-z])

def q_err(qr,q):
    return q_norm(q_mul(qr,q_conj(q)))

def q_to_dcm(q):
    q=q_norm(q); w,x,y,z=q
    return np.array([[1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)], [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]])

def gravity_gradient_torque(J, r_hat, mur3):
    return 3.0*mur3*np.cross(r_hat, J@r_hat)

def srp_torque(A,C,r_cp,sun_b):
    P=4.5e-6; return P*A*C*np.cross(r_cp, sun_b)

class SimParams:
    def __init__(self):
        import numpy as np
        self.J = np.diag([0.12,0.10,0.08]); self.Jinv=np.linalg.inv(self.J)
        self.r_leo=6771e3; self.mu=3.986004418e14
        self.srp_A=0.2; self.srp_C=1.2; self.r_cp=np.array([0.02,-0.01,0.0])
        self.h_rw_max=0.08; self.torque_max=0.06; self.dump_gain=0.005
class TimeSeries:
    def __init__(self):
        self.t=[]; self.q=[]; self.w=[]; self.tau=[]; self.h_rw=[]; self.err_angle=[]; self.point_b=[]

def propagate(t0, tf, dt, q0, w0, h0, ctrl_fun, ref_fun, params:SimParams, include_disturb=True):
    ts=TimeSeries(); q=q_norm(q0.copy()); w=w0.copy(); h=h0.copy()
    N=int(np.ceil((tf-t0)/dt))+1; t=t0
    for k in range(N):
        ref=ref_fun(t)
        q_ref=ref.get('q_ref', np.array([1,0,0,0])); w_ref=ref.get('w_ref', np.zeros(3))
        sun_i=ref.get('sun_i', np.array([1,0,0])); r_hat_i=ref.get('r_hat_i', np.array([0,1,0]))
        R=q_to_dcm(q); sun_b=R@sun_i; r_hat_b=R@r_hat_i
        tau_cmd=ctrl_fun(t,q,w,q_ref,w_ref,params)
        if np.linalg.norm(h)>params.h_rw_max: tau_cmd-=params.dump_gain*h
        tau=np.clip(tau_cmd,-params.torque_max,params.torque_max)
        tau_d=np.zeros(3)
        if include_disturb:
            mur3=params.mu/(params.r_leo**3)
            tau_d+=gravity_gradient_torque(params.J, r_hat_b, mur3)
            tau_d+=srp_torque(params.srp_A, params.srp_C, params.r_cp, sun_b)
        J=params.J; Jw=J@w; dw=params.Jinv@(tau+tau_d-np.cross(w,(Jw+h))); dh=-tau
        Omega=np.array([[0,-w[0],-w[1],-w[2]],[w[0],0,w[2],-w[1]],[w[1],-w[2],0,w[0]],[w[2],w[1],-w[0],0]])
        dq=0.5*Omega@q
        q_mid=q_norm(q+0.5*dt*dq); w_mid=w+0.5*dt*dw; h_mid=h+0.5*dt*dh
        Jw_mid=J@w_mid; dw_mid=params.Jinv@(tau+tau_d-np.cross(w_mid,(Jw_mid+h_mid)))
        dq_mid=0.5*np.array([[0,-w_mid[0],-w_mid[1],-w_mid[2]],[w_mid[0],0,w_mid[2],-w_mid[1]],[w_mid[1],-w_mid[2],0,w_mid[0]],[w_mid[2],w_mid[1],-w_mid[0],0]])@q_mid
        q=q_norm(q+dt*dq_mid); w=w+dt*dw_mid; h=h+dt*dh
        ts.t.append(t); ts.q.append(q.copy()); ts.w.append(w.copy()); ts.tau.append(tau.copy()); ts.h_rw.append(h.copy())
        qe=q_err(q_ref,q); angle=2*np.arctan2(np.linalg.norm(qe[1:]), abs(qe[0])+1e-12); ts.err_angle.append(angle)
        e1_i=R.T@np.array([1,0,0]); ts.point_b.append(e1_i.copy()); t+=dt
    ts.t=np.array(ts.t); ts.q=np.vstack(ts.q); ts.w=np.vstack(ts.w); ts.tau=np.vstack(ts.tau); ts.h_rw=np.vstack(ts.h_rw); ts.err_angle=np.array(ts.err_angle); ts.point_b=np.vstack(ts.point_b)
    return ts
