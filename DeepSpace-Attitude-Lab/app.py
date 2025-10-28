from __future__ import annotations
import streamlit as st
import numpy as np

from dynamics import SimParams, propagate
from controllers import detumble_controller, pid_att_controller, lqr_att_controller
from scenarios import ref_detumble, ref_sun_point, ref_nadir, ref_yaw_scan
from plotting import fig_time_series, fig_angle, fig_phase, fig_energy, fig_pointing_sphere, make_exam_pack, make_exam_ppt

st.set_page_config(page_title='DeepSpace Attitude Lab', layout='wide')
st.title('🛰️ DeepSpace Attitude Lab (DSAL)')
st.caption('Interactive spacecraft attitude control lab — Streamlit UI · 3D animation · 12+ figures · one-click Exam Pack & PPTX')

with st.sidebar:
    st.header('Simulation Setup')
    scenario_name = st.selectbox('Scenario', ['Safe-Mode Detumble', 'Sun-Pointing', 'Nadir-Pointing', 'Target Scan'])
    ctrl_name = st.selectbox('Controller', ['Detumble (PD on ω)', 'PID (quat+rate)', 'LQR (auto CARE)'])
    t_final = st.slider('Duration [s]', min_value=30, max_value=600, value=180, step=10)
    dt = st.slider('Step [s]', min_value=0.02, max_value=0.2, value=0.05, step=0.01)
    st.divider()
    st.subheader('Initial Conditions')
    w0_mag = st.slider('Initial rate |ω0| [deg/s]', 0.0, 10.0, 5.0, 0.5)
    tilt_deg = st.slider('Initial attitude tilt [deg]', 0.0, 90.0, 30.0, 5.0)
    st.divider()
    st.subheader('Limits & Disturbances')
    torque_max = st.slider('Max wheel torque [N·m]', 0.01, 0.2, 0.06, 0.01)
    hmax = st.slider('Wheel momentum limit [N·m·s]', 0.02, 0.2, 0.08, 0.01)
    use_dist = st.checkbox('Enable disturbances (GG + SRP)', value=True)

scenario_map = {
    'Safe-Mode Detumble': ref_detumble,
    'Sun-Pointing': ref_sun_point,
    'Nadir-Pointing': ref_nadir,
    'Target Scan': ref_yaw_scan
}
ctrl_map = {
    'Detumble (PD on ω)': detumble_controller,
    'PID (quat+rate)': pid_att_controller,
    'LQR (auto CARE)': lqr_att_controller
}

params = SimParams()
params.torque_max = float(torque_max)
params.h_rw_max = float(hmax)

# init states
w0 = np.array([w0_mag, -0.3*w0_mag, 0.2*w0_mag]) * np.pi/180.0
ang = np.deg2rad(tilt_deg)
q0 = np.array([np.cos(ang/2), np.sin(ang/2), 0.0, 0.0])  # rotate about x
h0 = np.array([0.0, 0.0, 0.0])

left, right = st.columns([1.2,1.0])
with left:
    if st.button('▶️ Run Simulation', use_container_width=True):
        with st.spinner('Simulating...'):
            ts = propagate(0.0, float(t_final), float(dt), q0, w0, h0,
                           ctrl_map[ctrl_name], scenario_map[scenario_name], params, include_disturb=use_dist)
        st.success('Done! Scroll for results ↓')
        st.session_state['ts'] = ts

if 'ts' in st.session_state:
    ts = st.session_state['ts']
    t = ts.t; q = ts.q; w = ts.w; tau = ts.tau; h = ts.h_rw; err = ts.err_angle; pts = ts.point_b

    tabs = st.tabs(['📊 Time Series','🧭 Error & Energy','🌀 Phase & Spectra','🌐 3D Animation','📦 Exports'])
    with tabs[0]:
        c1, c2 = st.columns(2)
        with c1:
            st.pyplot(fig_time_series(t, w, ['ωx','ωy','ωz'], 'Body Rates'))
            st.pyplot(fig_time_series(t, q[:,1:], ['qx','qy','qz'], 'Quaternion (vector part)'))
            st.pyplot(fig_time_series(t, np.linalg.norm(w,axis=1)[:,None], ['|ω|'], 'Rate Magnitude'))
        with c2:
            st.pyplot(fig_time_series(t, tau, ['τx','τy','τz'], 'Control Torque'))
            st.pyplot(fig_time_series(t, h, ['hx','hy','hz'], 'Wheel Momentum'))
            st.pyplot(fig_time_series(t, np.linalg.norm(tau,axis=1)[:,None], ['|τ|'], 'Torque Magnitude'))

    with tabs[1]:
        c3, c4 = st.columns(2)
        with c3:
            st.pyplot(fig_angle(t, err))
            st.pyplot(fig_energy(t, params.J, w))
        with c4:
            derr = np.gradient(err, t)
            st.pyplot(fig_time_series(t, derr[:,None], ['d(angle)/dt'], 'Error Angle Rate'))
            effort = np.cumsum(np.abs(tau), axis=0)*(t[1]-t[0])
            st.pyplot(fig_time_series(t, effort, ['Ex','Ey','Ez'], 'Cumulative Effort'))

    with tabs[2]:
        st.pyplot(fig_phase(w))

    with tabs[3]:
        st.plotly_chart(fig_pointing_sphere(pts), use_container_width=True)

    with tabs[4]:
        c5, c6 = st.columns(2)
        with c5:
            st.write('**Exam Pack (ZIP)** — 12 PNGs + a small GIF + summary.md')
            if st.button('📦 Build Exam Pack ZIP', type='primary', key='zip'):
                zip_bytes = make_exam_pack(t, q, w, tau, h, err, params.J, pts)
                st.download_button('Download Exam Pack (ZIP)', data=zip_bytes, file_name='DSAL_Exam_Pack.zip', mime='application/zip')
        with c6:
            st.write('**PPTX Export** — 12 figures auto-laid-out (2 per slide)')
            if st.button('🖼️ Build PPTX (12 figs)', type='secondary', key='ppt'):
                ppt_bytes = make_exam_ppt(t, q, w, tau, h, err, params.J)
                st.download_button('Download PPTX', data=ppt_bytes, file_name='DSAL_Results_12figs.pptx', mime='application/vnd.openxmlformats-officedocument.presentationml.presentation')
else:
    right = st.empty()
    right.info('点击左侧参数后，按下“Run Simulation”开始。默认参数即可出图。')
