"""
Active noise cancellation (ANC) simulation for a coaxial contra-rotating ducted fan.

What it does
  1. Builds the fan's tonal noise from blade counts and rpm:
       - rotor-alone tones          (cut off in a duct at subsonic tip speed)
       - lower-rotor / stator tones (Tyler-Sofrin modes m = n*B2 - k*V)
       - rotor / rotor tones        (modes m = n1*B1 - n2*B2)
     Each tone is kept loud if at least one of its modes propagates in the duct,
     and heavily attenuated if all its modes are cut off.
  2. Adds broadband "whoosh" noise.
  3. Runs rpm-synchronised FxLMS cancellation (the same family of algorithm used
     in ANC headphones and engine-order cancellation in cars) on the tones.
  4. Works out how many speakers a ring around the duct would need.
  5. Saves before/after WAV files, a spectrum plot and a summary.

Limits (be honest about these in your write-up)
  - Single error microphone: this shows cancellation at one point. Real in-duct
    ANC needs a speaker ring per propagating mode (step 4) to cancel everywhere.
  - Tone amplitudes are heuristic (relative, not calibrated dBA).
  - Broadband noise cannot be cancelled this way, and is left untouched.

Usage
  python anc_fan_sim.py                    # full-scale duct (830 mm)
  python anc_fan_sim.py --preset rig       # 150 mm bench rig
  python anc_fan_sim.py --b1 11 --b2 9 --stators 23 --rpm 2765 --diameter 0.83
"""
import argparse
import json
import numpy as np
from scipy.special import jnp_zeros
from scipy.signal import welch, butter, lfilter
from scipy.io import wavfile
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

C = 343.0
FS = 48000

PRESETS = {
    "full": dict(diameter=0.83, rpm=2765),
    "rig": dict(diameter=0.15, rpm=15300),
}


def jprime_first(m):
    """First zero of J'_m (hard-wall circular duct). m=0 -> plane wave (0)."""
    m = abs(int(m))
    return 0.0 if m == 0 else float(jnp_zeros(m, 1)[0])


def cut_on(m, f, radius):
    return 2 * np.pi * f / C * radius > jprime_first(m)


def propagating_modes(f, radius, m_max=60):
    """All (m, n) radial-azimuthal modes that propagate at frequency f."""
    kr = 2 * np.pi * f / C * radius
    modes = []
    for m in range(0, m_max + 1):
        zeros = [0.0] if m == 0 else []
        zeros += list(jnp_zeros(m, 10)) if m > 0 else list(jnp_zeros(0, 10))[0:9]
        n_on = sum(1 for z in zeros if z < kr)
        if n_on == 0 and m > 0:
            break
        for n in range(n_on):
            modes.append((m, n))
    return modes


def build_tones(b1, b2, v, rpm, radius, harmonics=3):
    """Return list of dicts: freq, label, modes, cut_on, amplitude."""
    shaft = rpm / 60.0
    tones = {}

    def add(freq, label, modes, base_amp):
        on = any(cut_on(m, freq, radius) for m in modes)
        amp = base_amp if on else base_amp * 10 ** (-25 / 20)   # cut-off -> -25 dB
        key = round(freq, 3)
        if key in tones:                                        # coincident tones add
            tones[key]["amplitude"] = np.hypot(tones[key]["amplitude"], amp)
            tones[key]["label"] += " + " + label
            tones[key]["cut_on"] = tones[key]["cut_on"] or on
            tones[key]["modes"] += modes
        else:
            tones[key] = dict(freq=freq, label=label, modes=list(modes), cut_on=on, amplitude=amp)

    for n in range(1, harmonics + 1):
        decay = 10 ** (-5 * (n - 1) / 20)
        add(n * b1 * shaft, f"upper rotor alone {n}xBPF", [n * b1], 0.6 * decay)
        add(n * b2 * shaft, f"lower rotor alone {n}xBPF", [n * b2], 0.6 * decay)
        st_modes = [n * b2 - k * v for k in range(-4, 5)]
        add(n * b2 * shaft, f"lower rotor/stators {n}xBPF", st_modes, 1.0 * decay)
    for n1 in (1, 2):
        for n2 in (1, 2):
            f = (n1 * b1 + n2 * b2) * shaft
            m = n1 * b1 - n2 * b2
            add(f, f"rotor/rotor ({n1},{n2})", [m], 0.8 * 10 ** (-5 * (n1 + n2 - 2) / 20))
    return sorted(tones.values(), key=lambda t: t["freq"])


def a_weight_db(f):
    f = np.maximum(f, 1e-3)
    ra = (12194 ** 2 * f ** 4) / ((f ** 2 + 20.6 ** 2) *
          np.sqrt((f ** 2 + 107.7 ** 2) * (f ** 2 + 737.9 ** 2)) * (f ** 2 + 12194 ** 2))
    return 20 * np.log10(ra) + 2.0


def dba_level(x):
    f, p = welch(x, FS, nperseg=8192)
    return 10 * np.log10(np.sum(p * 10 ** (a_weight_db(f) / 10)) + 1e-20)


def simulate(args):
    radius = args.diameter / 2
    tones = build_tones(args.b1, args.b2, args.stators, args.rpm, radius)
    n = int(args.seconds * FS)
    t = np.arange(n) / FS
    rng = np.random.default_rng(1)

    # slow +-0.5 % rpm wander: tones follow it, and so does the tachometer reference
    wander = 1 + 0.005 * np.sin(2 * np.pi * 0.3 * t)
    shaft_phase = 2 * np.pi * np.cumsum(args.rpm / 60.0 * wander) / FS

    # disturbance at the error mic
    d = np.zeros(n)
    orders = []
    for tone in tones:
        order = tone["freq"] / (args.rpm / 60.0)       # multiple of shaft frequency
        orders.append(order)
        d += tone["amplitude"] * np.cos(order * shaft_phase + rng.uniform(0, 2 * np.pi))
    white = rng.standard_normal(n)
    b, a = butter(1, 2000 / (FS / 2))                   # gently low-passed "whoosh"
    broadband = lfilter(b, a, white)
    broadband *= args.broadband / np.std(broadband)    # fixed level, same for every design
    d_total = d + broadband

    # FxLMS on rpm-synchronised sin/cos references, one weight pair per tone
    # speaker->mic path: 0.6 ms delay, gain 0.9. The controller's model of it is
    # deliberately imperfect: gain off by 10 % and phase off by --path-error degrees.
    delay_true, gain_true, gain_est = 0.6e-3, 0.9, 1.0
    orders = np.array(orders)
    w_true = orders[None, :] * (args.rpm / 60.0) * 2 * np.pi
    ph = orders[None, :] * shaft_phase[:, None]
    phase_err = np.radians(args.path_error) * np.where(np.arange(len(orders)) % 2, 1, -1)[None, :]
    p_true = ph - w_true * delay_true
    p_est = p_true + phase_err
    x_true = gain_true * np.concatenate([np.cos(p_true), np.sin(p_true)], 1)
    x_est = gain_est * np.concatenate([np.cos(p_est), np.sin(p_est)], 1)
    weights = np.zeros(x_true.shape[1])
    mu = args.mu
    err = np.empty(n)
    on_at = int(args.anc_on * FS)
    for i in range(n):
        y = x_true[i] @ weights if i >= on_at else 0.0
        e = d_total[i] + y
        err[i] = e
        if i >= on_at:
            weights -= mu * e * x_est[i]

    settle = on_at + int(1.0 * FS)
    before, after = d_total[:on_at], err[settle:]
    red_total = dba_level(before) - dba_level(after)
    red_tonal = dba_level(d[:on_at]) - dba_level(err[settle:] - broadband[settle:])

    # speaker ring sizing: only the modes each tone actually excites and that escape
    ring = []
    for tone in tones:
        if not tone["cut_on"]:
            continue
        excited = sorted({abs(m) for m in tone["modes"] if cut_on(m, tone["freq"], radius)})
        all_modes = propagating_modes(tone["freq"], radius)
        radial = max(sum(1 for mm, _ in all_modes if mm == m) for m in excited)
        m_max = max(excited)
        ring.append(dict(freq=round(tone["freq"], 1), escaping_azimuthal_orders=excited,
                         speakers_per_ring=2 * m_max + 1, rings_needed=radial))
    level_before = dba_level(before)

    # outputs
    tag = f"B{args.b1}-{args.b2}_V{args.stators}_D{int(args.diameter*1000)}"
    peak = max(np.max(np.abs(d_total)), 1e-9) * 1.1
    wavfile.write(f"fan_{tag}_before.wav", FS, (before / peak * 32767).astype(np.int16))
    wavfile.write(f"fan_{tag}_anc_on.wav", FS, (err / peak * 32767).astype(np.int16))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    levels = []
    for sig, lab, col in ((before, "ANC off", "#888780"), (after, "ANC on", "#2a78d6")):
        f, p = welch(sig, FS, nperseg=16384)
        lv = 10 * np.log10(p + 1e-20)
        levels.append(lv[(f > 50) & (f < 12000)])
        ax1.semilogx(f, lv, color=col, lw=1.2, label=lab)
    ax1.set_ylim(np.median(levels[0]) - 25, np.max(levels[0]) + 5)
    ax1.set_xlim(50, 12000); ax1.set_xlabel("Frequency (Hz)"); ax1.set_ylabel("Level (dB, relative)")
    ax1.set_title(f"{args.b1}/{args.b2} blades, {args.stators} stators, {args.diameter*1000:.0f} mm duct, "
                  f"{args.rpm:.0f} rpm  |  tonal -{red_tonal:.1f} dBA, overall -{red_total:.1f} dBA")
    ax1.legend(); ax1.grid(alpha=0.3, which="both")
    win = int(0.05 * FS)
    env = np.sqrt(np.convolve(err ** 2, np.ones(win) / win, mode="valid"))
    ax2.plot(t[win // 2: win // 2 + len(env)], 20 * np.log10(env + 1e-12), color="#2a78d6", lw=1)
    ax2.axvline(args.anc_on, color="#d85a30", ls="--", lw=1, label="ANC switched on")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Mic level (dB, relative)")
    ax2.set_title("Convergence at the error microphone"); ax2.legend(); ax2.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"fan_{tag}_anc.png", dpi=130)

    summary = dict(
        inputs=vars(args),
        tones=[dict(freq_hz=round(x["freq"], 1), source=x["label"], escapes_duct=x["cut_on"],
                    relative_amp=round(float(x["amplitude"]), 3)) for x in tones],
        level_anc_off_dBA_relative=round(level_before, 1),
        reduction_dBA=dict(tonal_only=round(red_tonal, 1), overall_including_broadband=round(red_total, 1)),
        speaker_ring=ring,
    )
    with open(f"fan_{tag}_summary.json", "w") as fh:
        json.dump(summary, fh, indent=2)

    print(f"\n{tag}")
    print(f"{'Hz':>8}  {'escapes':>7}  source")
    for x in tones:
        print(f"{x['freq']:8.1f}  {'yes' if x['cut_on'] else 'no':>7}  {x['label']}")
    print(f"\nLevel with ANC off: {level_before:.1f} dBA (relative scale, comparable between designs)")
    print(f"ANC reduction: tones -{red_tonal:.1f} dBA, overall -{red_total:.1f} dBA (broadband untouched)")
    print("Speaker ring needed for the escaping tones:")
    for r in ring:
        print(f"  {r['freq']:7.1f} Hz: modes m={r['escaping_azimuthal_orders']} -> "
              f"{r['speakers_per_ring']} speakers/ring x {r['rings_needed']} ring(s)")
    print(f"Saved fan_{tag}_before.wav, fan_{tag}_anc_on.wav, fan_{tag}_anc.png, fan_{tag}_summary.json")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", choices=PRESETS, default="full")
    ap.add_argument("--b1", type=int, default=11, help="upper rotor blades")
    ap.add_argument("--b2", type=int, default=9, help="lower rotor blades")
    ap.add_argument("--stators", type=int, default=23)
    ap.add_argument("--rpm", type=float)
    ap.add_argument("--diameter", type=float, help="duct tip diameter, m")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--anc-on", type=float, default=2.0, help="time ANC switches on, s")
    ap.add_argument("--broadband", type=float, default=0.35, help="broadband level vs tones (rms ratio)")
    ap.add_argument("--mu", type=float, default=0.0015, help="FxLMS step size")
    ap.add_argument("--path-error", type=float, default=25.0,
                    help="phase error (deg) in the controller's speaker-to-mic model; >90 diverges")
    args = ap.parse_args()
    p = PRESETS[args.preset]
    args.rpm = args.rpm or p["rpm"]
    args.diameter = args.diameter or p["diameter"]
    simulate(args)


if __name__ == "__main__":
    main()
