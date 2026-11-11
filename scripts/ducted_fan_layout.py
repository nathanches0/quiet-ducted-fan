"""
Ducted coaxial fan - sizing, 1:10 scale geometry, blade/stator acoustics check,
and SolidWorks global-variable export.

Run:  python ducted_fan_layout.py
Output: console report + sw_globals_1to10.txt (link it in SolidWorks:
        Tools > Equations > "Link to external file").

All lengths in mm, angles in degrees (set your SolidWorks part to MMGS).
"""
import math

# ---------------------------------------------------------------- INPUTS
RHO = 1.225            # air density, kg/m^3 (sea level, ISA)
C_SOUND = 343.0        # speed of sound, m/s
G = 9.81

MTOW_KG = 200.0
N_DUCTS = 4
CANT_DEG = 12.0       # outward cant of each duct axis (exhaust away from rider)
HOVER_ELEC_KW = 40.0   # total electrical budget in hover (your spec)
PEAK_THRUST_N = 3000.0
PEAK_ELEC_KW = 60.0

FM_DUCT = 0.75         # figure of merit vs IDEAL ducted fan (0.7-0.8 is good practice)
ETA_DRIVE = 0.90       # motor x ESC efficiency
SIGMA_EXIT = 1.10      # diffuser exit-area / rotor-area ratio
HUB_RATIO = 0.22       # hub diameter / tip diameter (coax motors live in the hub)

TIP_SPEED = 120.0      # m/s  - "low RPM" target, tip Mach ~0.35
B_UPPER = 11           # upper rotor blades
B_LOWER = 9            # lower rotor blades (decided: 9, see design log)
V_STATORS = 23         # stators under the lower rotor (decided: 23)

SOLIDITY = 0.45        # rotor solidity B*c/(pi*R) at mid-span
LIP_R_FRAC = 0.10      # inlet lip radius / tip diameter (0.08-0.13 typical for hover)
DUCT_L_FRAC = 0.60     # duct length / tip diameter
ROTOR1_DEPTH_FRAC = 0.12  # upper rotor plane below lip highlight, / D
GAP_CHORDS = 1.5       # axial gaps (rotor-rotor, rotor-stator) in chords
TIP_GAP_FULL = 0.5     # mm, your full-scale target

SCALE = 0.1
PRINT_MIN_WALL = 2.0   # mm, practical SLA/FDM skin for a 1:10 duct


def ideal_ducted_power(thrust_n, area_m2, sigma=1.0):
    """Momentum theory, ducted rotor: P = T^1.5 / sqrt(4 sigma rho A)."""
    return thrust_n ** 1.5 / math.sqrt(4.0 * sigma * RHO * area_m2)


def area_for_power(thrust_n, p_ideal_w, sigma=1.0):
    return thrust_n ** 3 / (4.0 * sigma * RHO * p_ideal_w ** 2)


def jprime(m):
    """First zero of J'_m (hard-wall circular duct), approx for m>=2."""
    table = {0: 0.0, 1: 1.8412, 2: 3.0542, 3: 4.2012, 4: 5.3176}
    if m in table:
        return table[m]
    return m + 0.8086 * m ** (1 / 3) + 0.072 * m ** (-1 / 3)


def tyler_sofrin(B, V, mach_tip, n_max=3, k_range=4):
    """Rotor-stator interaction modes m = nB - kV. A mode is cut-on (propagates)
    if n*B*M_tip > j'_|m|,1 (hub-less approximation)."""
    rows = []
    for n in range(1, n_max + 1):
        for k in range(-k_range, k_range + 1):
            m = n * B - k * V
            if abs(m) > 40:
                continue
            on = n * B * mach_tip > jprime(abs(m))
            rows.append((n, k, m, on))
    return rows


def best_clock(count, half_width_deg, mode="clear"):
    """Choose a clocking offset for a 180-degree cutaway (cut plane at 0/180 deg).
    clear    -> maximise clearance so no element is sliced
    showcase -> put one element centred on the 0-deg plane"""
    pitch = 360.0 / count
    if mode == "showcase":
        return 0.0, 0.0
    best = (-1e9, 0.0)
    steps = 2000
    for i in range(steps):
        phi = pitch * i / steps
        clearance = 1e9
        for j in range(count):
            a = (phi + j * pitch) % 360.0
            for plane in (0.0, 180.0, 360.0):
                clearance = min(clearance, abs(a - plane) - half_width_deg)
        if clearance > best[0]:
            best = (clearance, phi)
    return best[1], best[0]


def main():
    W = MTOW_KG * G
    T_duct = W / (N_DUCTS * math.cos(math.radians(CANT_DEG)))  # axial thrust per duct
    p_elec = HOVER_ELEC_KW * 1000 / N_DUCTS
    p_ideal = p_elec * ETA_DRIVE * FM_DUCT
    A_net = area_for_power(T_duct, p_ideal, SIGMA_EXIT)
    A_gross = A_net / (1 - HUB_RATIO ** 2)
    D = math.sqrt(4 * A_gross / math.pi)           # m
    R = D / 2

    print("=== FULL-SCALE SIZING (hover budget drives diameter) ===")
    print(f"Hover weight            {W:8.0f} N  ({T_duct:.0f} N axial per duct, {N_DUCTS} ducts at {CANT_DEG:.0f} deg cant)")
    print(f"Cant power penalty      {(1/math.cos(math.radians(CANT_DEG)))**1.5*100-100:8.1f} %")
    print(f"Electrical per duct     {p_elec/1000:8.1f} kW -> ideal-equivalent {p_ideal/1000:.1f} kW")
    print(f"Required net disc area  {A_net:8.3f} m^2 per duct")
    print(f"Tip diameter            {D*1000:8.0f} mm per duct")

    # peak-thrust check at that diameter
    T_peak_duct = PEAK_THRUST_N / (N_DUCTS * math.cos(math.radians(CANT_DEG)))
    p_peak = N_DUCTS * ideal_ducted_power(T_peak_duct, A_net, SIGMA_EXIT) \
        / (ETA_DRIVE * FM_DUCT) / 1000
    print(f"Power for {PEAK_THRUST_N:.0f} N peak    {p_peak:8.1f} kW  (spec budget {PEAK_ELEC_KW:.0f} kW)")

    # what narrow ducts would cost
    print("\nNarrow-duct penalty (hover electrical power, all ducts):")
    for d_try in (0.4, 0.6, D):
        a = math.pi * (d_try / 2) ** 2 * (1 - HUB_RATIO ** 2)
        p = N_DUCTS * ideal_ducted_power(T_duct, a, SIGMA_EXIT) / (ETA_DRIVE * FM_DUCT) / 1000
        print(f"  D = {d_try*1000:5.0f} mm  ->  {p:6.1f} kW   ({W/(p*1000)*1000:5.1f} N/kW)")

    # duct-lip share: ideal ducted theory, sigma=1 -> duct carries 1/2; general 1 - 1/(2 sigma)
    duct_share = 1 - 1 / (2 * SIGMA_EXIT)
    # rotor-plane velocity: T = rho*A*v_r*w, w = v_r/sigma  ->  v_r = sqrt(sigma*T/(rho*A))
    v_rotor = math.sqrt(SIGMA_EXIT * T_duct / (RHO * A_net))
    print(f"\nIdeal duct (shroud) thrust share: {duct_share*100:.0f}%  "
          "<- already inside the momentum-theory numbers above, not an extra bonus")

    rpm = TIP_SPEED / (math.pi * D) * 60
    m_tip = TIP_SPEED / C_SOUND
    c = SOLIDITY * math.pi * (R * 0.75) / B_UPPER       # chord at 75% R, m
    print(f"Rotor speed {rpm:.0f} rpm, tip Mach {m_tip:.2f}, chord ~{c*1000:.0f} mm")

    # ---------------------------------------------------- 1:10 GEOMETRY (mm)
    Dm = D * 1000 * SCALE
    Rm = Dm / 2
    lip_r = LIP_R_FRAC * Dm
    L = DUCT_L_FRAC * Dm
    cm = c * 1000 * SCALE
    z_r1 = ROTOR1_DEPTH_FRAC * Dm
    z_r2 = z_r1 + GAP_CHORDS * cm
    z_st = z_r2 + GAP_CHORDS * cm
    R_exit = Rm * math.sqrt(SIGMA_EXIT)
    diff_angle = math.degrees(math.atan((R_exit - Rm) / (L - z_r2)))
    tip_gap_model = max(TIP_GAP_FULL * SCALE, 0.3)

    geo = {
        "D_tip": Dm, "R_tip": Rm, "R_hub": Rm * HUB_RATIO,
        "R_bore": Rm + tip_gap_model,
        "lip_radius": lip_r, "lip_highlight_R": Rm + tip_gap_model + lip_r,
        "duct_length": L, "R_exit": R_exit + tip_gap_model,
        "diffuser_half_angle": diff_angle,
        "z_rotor_upper": z_r1, "z_rotor_lower": z_r2, "z_stator_LE": z_st,
        "chord": cm, "blade_t_max": 0.10 * cm,
        "tip_gap": tip_gap_model, "skin_min": PRINT_MIN_WALL,
        "N_blades_upper": B_UPPER, "N_blades_lower": B_LOWER,
        "N_stators": V_STATORS,
        "pitch_upper": 360.0 / B_UPPER, "pitch_lower": 360.0 / B_LOWER,
        "pitch_stator": 360.0 / V_STATORS,
    }

    # angular half-width of each element at the tip (for cutaway clearance)
    hw_blade = math.degrees(0.5 * cm / Rm) * 0.7   # stagger reduces projected width
    hw_stator = math.degrees(0.5 * 0.15 * cm / Rm) # thin strut (~15% chord thickness)
    geo["clock_upper"], cl_u = best_clock(B_UPPER, hw_blade)
    geo["clock_lower"], cl_l = best_clock(B_LOWER, hw_blade)
    geo["clock_stator"], cl_s = best_clock(V_STATORS, hw_stator)

    print("\n=== 1:10 STATIC MODEL (mm, deg) ===")
    for k, v in geo.items():
        print(f"  {k:22s} {v:9.2f}")
    print(f"  cutaway clearance: upper {cl_u:.1f} deg, lower {cl_l:.1f} deg, stators {cl_s:.1f} deg")

    re_full = v_rotor * (LIP_R_FRAC * D) / 1.5e-5
    print(f"\nLip Reynolds number: full-scale ~{re_full:.1e}, 1:10 at same velocity ~{re_full*SCALE:.1e}")

    # ---------------------------------------------------- ACOUSTICS CHECK
    print("\n=== TYLER-SOFRIN ROTOR-STATOR MODES (lower rotor vs stators) ===")
    for V in (V_STATORS, 2 * B_LOWER + 1, 2 * B_LOWER + 3):
        rows = tyler_sofrin(B_LOWER, V, m_tip, n_max=2)
        on = [(n, m) for n, k, m, cut_on in rows if cut_on]
        tag = " <- current" if V == V_STATORS else ""
        print(f"  B={B_LOWER}, V={V:2d}: propagating (BPF harmonic, mode) = {on or 'none at 1-2xBPF'}{tag}")
    m_rr = B_UPPER - B_LOWER
    print(f"  Rotor-rotor: m = {B_UPPER}-{B_LOWER} = {m_rr}"
          + ("  <- plane wave, always propagates; use different counts" if m_rr == 0 else ""))

    # ---------------------------------------------------- 1:10 VEHICLE LAYOUT (mm)
    # x forward, y left, z up; origin at centre of mass on the body centreline
    body_w = 400 * SCALE             # central body / seat width at full scale 400 mm
    pod_gap = 0.05 * Dm              # clearance between duct outer lip and body
    duct_outer_R = geo["lip_highlight_R"] + PRINT_MIN_WALL
    y_pod = body_w / 2 + pod_gap + duct_outer_R
    x_pod = duct_outer_R + 0.10 * Dm  # front/rear pair spacing = 2*x_pod
    geo.update({
        "cant_deg": CANT_DEG,
        "duct_outer_R": duct_outer_R,
        "body_width": body_w,
        "pod_y": y_pod, "pod_x_front": x_pod, "pod_x_rear": -x_pod,
        "overall_width": 2 * (y_pod + duct_outer_R),
        "overall_length": 2 * (x_pod + duct_outer_R),
    })
    print("\n=== 1:10 VEHICLE LAYOUT (mm) ===")
    for k in ("cant_deg", "duct_outer_R", "pod_y", "pod_x_front", "pod_x_rear",
              "overall_width", "overall_length"):
        print(f"  {k:22s} {geo[k]:9.2f}")
    print("  Spin directions: front-left CW-top, front-right CCW-top, rear-left CCW-top, rear-right CW-top")

    # ---------------------------------------------------- EXPORT
    with open("sw_globals_1to10.txt", "w") as f:
        f.write("' Ducted coax fan, 1:10 static model. Units: mm / deg. Generated by ducted_fan_layout.py\n")
        for k, v in geo.items():
            val = str(v) if isinstance(v, int) else f"{v:.4f}"
            f.write(f'"{k}"= {val}\n')
    print("\nWrote sw_globals_1to10.txt")


if __name__ == "__main__":
    main()
