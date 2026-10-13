import numpy as np
import scipy.signal as sig
import scipy.linalg as la
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- Parameters (DIATONE, from DRONE_PARAM_SUSPENDED_LOAD) ----
m   = 0.762       # drone mass [kg] (DIATONE, acsl-tcu/common_matlab)
mL  = 0.0556      # load mass  [kg]
L   = 0.46        # cable length [m]
g   = 9.81
Jy  = 0.02985236  # pitch inertia [kg m^2]
f0  = (m+mL)*g    # hover thrust [N]

wn2 = (m+mL)*g/(m*L)      # pendulum natural freq^2 (drone mass m & cable L; NOT load mass mL)
wn  = np.sqrt(wn2)
print(f"hover thrust f0 = {f0:.4f} N")
print(f"pendulum wn = {wn:.4f} rad/s = {wn/2/np.pi:.4f} Hz")

# small physical damping
zeta_p = 0.03     # pendulum aero damping ratio
c_att  = 0.02     # attitude/rotor drag (small) -> moves origin poles slightly into LHP

# ---- State-space  x = [beta, betadot, theta, thetadot], u = tau, y = theta ----
# betadot2 = (tau - c_att*betadot)/Jy
# theta2   = -(f0/(mL*L))*beta - wn2*theta - 2*zeta_p*wn*thetadot
#          = -wn2*beta - wn2*theta - 2 zeta_p wn thetadot   (since f0/(mL L)=wn2)
A = np.array([
    [0,            1,        0,          0],
    [0,     -c_att/Jy,       0,          0],
    [0,            0,        0,          1],
    [-wn2,         0,     -wn2,   -2*zeta_p*wn],
])
B = np.array([[0],[1/Jy],[0],[0]])
C = np.array([[0,0,1,0]])       # output = swing angle theta [rad]
D = np.array([[0]])
P = sig.StateSpace(A,B,C,D)
poles = np.linalg.eigvals(A)
print("plant poles:", np.round(poles,4))

# ---- Bode ----
w = np.logspace(-1, 2.3, 2000)
_, mag, ph = sig.bode(P, w)

# ---- initial-condition response: release load from 10 deg swing (shows the wobble) ----
t = np.linspace(0, 20, 4000)
x0 = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])
_, y_ol, _ = sig.lsim((A, B, C, D), U=np.zeros_like(t), T=t, X0=x0)

# ---- LQR active damping (concept demo; MATLAB uses Hinf mixsyn) ----
Q = np.diag([1.0, 0.1, 50.0, 5.0])   # penalize swing angle & rate
R = np.array([[2.0]])
Sr = la.solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ Sr)     # u = -K x
Acl = A - B@K
print("closed-loop poles:", np.round(np.linalg.eigvals(Acl),4))
_, y_cl, _ = sig.lsim((Acl, B, C, D), U=np.zeros_like(t), T=t, X0=x0)
Pcl = sig.StateSpace(Acl, B, C, D)
_, magc, phc = sig.bode(Pcl, w)

# damping ratios of dominant modes
def dratio(p):
    return -np.real(p)/np.abs(p) if np.abs(p)>1e-9 else 1.0
ol_zeta = min(dratio(p) for p in poles if abs(p.imag)>1e-6)
cl_zetas = [dratio(p) for p in np.linalg.eigvals(Acl) if abs(p.imag)>1e-6]
print(f"open-loop pendulum zeta   = {ol_zeta:.3f}")
print(f"closed-loop min osc zeta  = {min(cl_zetas):.3f}")

# ================= Figure =================
plt.rcParams.update({"font.size":11,"axes.grid":True,"grid.alpha":0.3})
fig = plt.figure(figsize=(12,7.5), constrained_layout=True)
gs = fig.add_gridspec(2,2)

axm = fig.add_subplot(gs[0,0])
axm.semilogx(w, mag, lw=2, label="plant P (open)")
axm.semilogx(w, magc, lw=2, ls="--", label="damped (active)")
axm.axvline(wn, color="crimson", ls=":", lw=1.4)
axm.text(wn*1.05, axm.get_ylim()[0]+8, f"$\\omega_n$={wn:.2f} rad/s\n({wn/2/np.pi:.2f} Hz)", color="crimson", fontsize=9)
axm.set_ylabel("Magnitude [dB]"); axm.set_title("Bode: torque $\\tau_y$ → swing angle $\\theta$")
axm.legend(fontsize=9)

axp = fig.add_subplot(gs[1,0])
axp.semilogx(w, ph, lw=2)
axp.semilogx(w, phc, lw=2, ls="--")
axp.axvline(wn, color="crimson", ls=":", lw=1.4)
axp.set_ylabel("Phase [deg]"); axp.set_xlabel("Frequency [rad/s]")

axi = fig.add_subplot(gs[:,1])
axi.plot(t, np.rad2deg(y_ol), lw=2, label=f"open loop — wobbles for ~20 s (ζ≈{ol_zeta:.02f})")
axi.plot(t, np.rad2deg(y_cl), lw=2, label=f"active damping — settles in ~2 s (ζ≈{min(cl_zetas):.2f})")
axi.axhline(0, color="k", lw=0.6)
axi.set_xlabel("time [s]"); axi.set_ylabel("swing angle θ [deg]")
axi.set_title("Load released from 10° swing (open loop vs. active)")
axi.legend()

fig.suptitle("Single-drone suspended-load: system-ID model & active sway suppression (concept preview)", fontweight="bold")
fig.savefig("/tmp/work/preview_sway.png", dpi=130)
print("saved preview")
