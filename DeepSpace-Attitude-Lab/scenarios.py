import numpy as np
SUN_I = np.array([1.0,0.0,0.0]); RADIAL_I = np.array([0.0,1.0,0.0])
def align_body_x_to(v_i):
    v = v_i/(np.linalg.norm(v_i)+1e-12); b=np.array([1,0,0])
    axis = np.cross(b,v); c=float(np.dot(b,v)); s=float(np.linalg.norm(axis))
    if s < 1e-8: return np.array([1,0,0,0]) if c>0 else np.array([0,0,0,1])
    axis/=s; ang=np.arctan2(s,c); h=0.5*ang; return np.array([np.cos(h), *(np.sin(h)*axis)])
def ref_detumble(t): return {'q_ref':np.array([1,0,0,0]), 'w_ref':np.zeros(3), 'sun_i':SUN_I, 'r_hat_i':RADIAL_I}
def ref_sun_point(t): return {'q_ref':align_body_x_to(SUN_I), 'w_ref':np.zeros(3), 'sun_i':SUN_I, 'r_hat_i':RADIAL_I}
def ref_nadir(t): return {'q_ref':align_body_x_to(-RADIAL_I), 'w_ref':np.zeros(3), 'sun_i':SUN_I, 'r_hat_i':RADIAL_I}
def ref_yaw_scan(t):
    yaw = 0.5*np.pi/180.0*t; cy,sy=np.cos(0.5*yaw), np.sin(0.5*yaw); qz=np.array([cy,0,0,sy])
    return {'q_ref':qz, 'w_ref':np.array([0,0,0.5*np.pi/180.0]), 'sun_i':SUN_I, 'r_hat_i':RADIAL_I}
