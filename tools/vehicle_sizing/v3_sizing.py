#!/usr/bin/env python3
"""
v3_sizing.py — first-order sizing of Vehicle Version 3 (series-hybrid,
high-speed hovercraft), with Version 2 alongside for comparison.

    python3 tools/vehicle_sizing/v3_sizing.py            # print the report
    python3 tools/vehicle_sizing/v3_sizing.py --md FILE  # also write it as Markdown

Every number is a stated design estimate from simple, textbook models, not
validated data. Change the inputs in DESIGNS below and rerun. The models are:

  cushion      p = W / A;  lift air Q = perimeter * gap * Cd * sqrt(2p/rho_air)
               lift power = p * Q / eta, times a margin
  hump drag    over water the cushion's depression makes waves; the peak
               ("hump") comes at Froude number ~0.56 (v = 0.56 sqrt(g L)):
               R_hump / W = C_w * p / (rho_water g L), C_w ~ 1.5
  drag         air 0.5 rho CdA v^2 + air intake (momentum) rho Q v + skirt/spray
               c_skirt * W + residual wave drag above the hump
  fans         ducted-fan momentum theory: T = rho A vj (vj - v),
               shaft power = 0.5 rho A vj (vj^2 - v^2) / eta_fan
  braking      reverse thrust (a fraction of static thrust) plus drag
  turning      radius = v^2 / (mu_lat g), mu_lat = sideways force / weight
  hybrid       diesel genset: fuel = electrical power / eta_gen * BSFC
"""
import argparse
import math
import sys

RHO_AIR, RHO_WATER, G = 1.2, 1000.0, 9.81
KMH = 1 / 3.6

# --------------------------------------------------------------------------
# Inputs. Masses in kg, lengths in m, powers in W.
# --------------------------------------------------------------------------
DESIGNS = {
    "Version 2": dict(
        length=2.5, beam=1.5, corner_r=0.39,
        mass={"hull and skirt": 70, "tracks": 70, "fans and motors": 50,
              "10 kWh battery": 65, "electronics": 15, "payload": 30},
        fan_d=0.50, fans=2, fan_shaft_max=3000, reverse_frac=0.8,
        cd_a=0.9 * 1.5 * 0.9,           # Cd 0.9 (bluff, open payload) x frontal 1.5 x 0.9 m
        hotel=150, power="battery", battery_kwh=10.0, fuel_l=0.0,
        mu_lat=0.10,
    ),
    "Version 3": dict(
        length=3.0, beam=1.8, corner_r=0.39,
        mass={"hull and skirt (larger)": 90, "tracks": 70,
              "fans, motors, rudders, puff ports": 55,
              "diesel engine ~20 kW": 80, "generator": 25,
              "cooling, exhaust, mounts": 20, "fuel 30 L + tank": 30,
              "buffer battery 5 kWh": 40, "electronics": 20, "payload": 100},
        fan_d=0.70, fans=2, fan_shaft_max=10000, reverse_frac=0.8,
        cd_a=0.55 * 1.8 * 1.0,          # Cd 0.55 (faired bow, enclosed payload) x 1.8 x 1.0 m
        hotel=300, power="series hybrid", battery_kwh=5.0, fuel_l=30.0,
        mu_lat=0.10,
    ),
}

COMMON = dict(
    skirt_gap=0.006, skirt_cd=0.6, lift_eta=0.5, lift_margin=1.5,
    c_wave=1.5, c_skirt=0.02, eta_fan=0.75, eta_motor=0.92,
    eta_gen=0.92, bsfc=0.250, diesel_kg_per_l=0.84,     # kg/kWh
    headwind=5.0, reaction_s=1.0, lift_dump_decel=3.0,
    track_rr=0.06, track_share=0.6, track_speed=15 * KMH,
)
TARGETS = dict(cruise=30 * KMH, top=50 * KMH, slow=10 * KMH)


def geometry(d):
    r = d["corner_r"]
    area = d["length"] * d["beam"] - (4 - math.pi) * r ** 2
    perimeter = 2 * (d["length"] + d["beam"]) - (8 - 2 * math.pi) * r
    return area, perimeter


def fan_area(d):
    return d["fans"] * math.pi * (d["fan_d"] / 2) ** 2


def jet_speed(thrust, v, area):
    """Jet speed a ducted fan needs to make `thrust` at forward speed v."""
    return v / 2 + math.sqrt(v * v / 4 + thrust / (RHO_AIR * area))


def fan_shaft_power(thrust, v, area):
    if thrust <= 0:
        return 0.0
    vj = jet_speed(thrust, v, area)
    return 0.5 * RHO_AIR * area * vj * (vj * vj - v * v) / COMMON["eta_fan"]


def thrust_available(v, d):
    """Largest total thrust the fans make at speed v within their shaft power."""
    area, p_max = fan_area(d), d["fans"] * d["fan_shaft_max"]
    lo, hi = 0.0, 20000.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if fan_shaft_power(mid, v, area) <= p_max else (lo, mid)
    return lo


class Craft:
    def __init__(self, name, d):
        self.name, self.d = name, d
        self.m = sum(d["mass"].values())
        self.w = self.m * G
        self.area, self.perim = geometry(d)
        self.p = self.w / self.area
        self.q = (self.perim * COMMON["skirt_gap"] * COMMON["skirt_cd"]
                  * math.sqrt(2 * self.p / RHO_AIR))
        self.lift_w = self.p * self.q / COMMON["lift_eta"] * COMMON["lift_margin"]
        self.v_hump = 0.56 * math.sqrt(G * d["length"])
        self.r_hump = COMMON["c_wave"] * self.p / (RHO_WATER * G * d["length"]) * self.w

    def drag(self, v, wind=0.0):
        """Resistance over water above the hump (N)."""
        air = 0.5 * RHO_AIR * self.d["cd_a"] * (v + wind) ** 2
        intake = RHO_AIR * self.q * v
        skirt = COMMON["c_skirt"] * self.w
        wave = self.r_hump * 0.6 * min(1.0, (self.v_hump / max(v, 1e-6)) ** 2)
        return air + intake + skirt + wave

    def top_speed(self, wind=0.0):
        v = 0.5
        while v < 40 and thrust_available(v, self.d) > self.drag(v, wind):
            v += 0.05
        return v

    def elec_power(self, v, wind=0.0):
        """Electrical power (W) cruising at v: lift + thrust + hotel."""
        thrust = fan_shaft_power(self.drag(v, wind), v, fan_area(self.d))
        return self.lift_w + thrust / COMMON["eta_motor"] + self.d["hotel"]

    def endurance_h(self, v):
        p = self.elec_power(v)
        if self.d["power"] == "battery":
            return 0.9 * self.d["battery_kwh"] * 1000 / p
        fuel_kg_h = p / COMMON["eta_gen"] / 1000 * COMMON["bsfc"]
        return self.d["fuel_l"] * COMMON["diesel_kg_per_l"] / fuel_kg_h

    def fuel_l_h(self, v):
        return (self.elec_power(v) / COMMON["eta_gen"] / 1000 * COMMON["bsfc"]
                / COMMON["diesel_kg_per_l"])

    def stop_distance(self, v0, lift_dump=False):
        """Reaction distance + braking distance with full reverse thrust."""
        reverse = self.d["reverse_frac"] * thrust_available(0.0, self.d)
        dist, v, dt = v0 * COMMON["reaction_s"], v0, 0.01
        while v > 0:
            a = COMMON["lift_dump_decel"] if lift_dump else (reverse + self.drag(v)) / self.m
            dist += v * dt
            v -= a * dt
        return dist

    def turn_radius(self, v, mu=None):
        return v * v / ((mu or self.d["mu_lat"]) * G)


def report():
    crafts = [Craft(n, d) for n, d in DESIGNS.items()]
    v2, v3 = crafts
    L = []
    row = lambda label, f, fmt: L.append(f"| {label} | " + " | ".join(fmt.format(f(c)) for c in crafts) + " |")
    L.append("| Quantity | " + " | ".join(c.name for c in crafts) + " |")
    L.append("|---|" + "---|" * len(crafts))
    row("Footprint (m)", lambda c: f"{c.d['length']} x {c.d['beam']}", "{}")
    row("Total mass incl. payload (kg)", lambda c: c.m, "{:.0f}")
    row("Payload (kg)", lambda c: c.d["mass"]["payload"], "{:.0f}")
    row("Cushion pressure (kPa)", lambda c: c.p / 1000, "{:.2f}")
    row("Lift power, electrical, with margin (kW)", lambda c: c.lift_w / 1000, "{:.1f}")
    row("Hump speed over water (km/h)", lambda c: c.v_hump * 3.6, "{:.1f}")
    row("Wave drag at the hump (N)", lambda c: c.r_hump, "{:.0f}")
    row("Static thrust, both fans (N)", lambda c: thrust_available(0.0, c.d), "{:.0f}")
    row("Static thrust / weight", lambda c: thrust_available(0.0, c.d) / c.w, "{:.2f}")
    row("Thrust / hump resistance", lambda c: thrust_available(c.v_hump, c.d) / (
        c.r_hump + c.drag(c.v_hump) - c.r_hump * 0.6), "{:.1f}x")
    row("Top speed, calm (km/h)", lambda c: c.top_speed() * 3.6, "{:.0f}")
    row("Top speed, 5 m/s headwind (km/h)", lambda c: c.top_speed(COMMON["headwind"]) * 3.6, "{:.0f}")
    row("Drag at 30 km/h (N)", lambda c: c.drag(TARGETS["cruise"]), "{:.0f}")
    row("Drag at 50 km/h (N)", lambda c: c.drag(TARGETS["top"]), "{:.0f}")
    row("Electrical power at 10 km/h (kW)", lambda c: c.elec_power(TARGETS["slow"]) / 1000, "{:.1f}")
    row("Electrical power at 30 km/h (kW)", lambda c: c.elec_power(TARGETS["cruise"]) / 1000, "{:.1f}")
    row("Electrical power at 50 km/h (kW)", lambda c: c.elec_power(TARGETS["top"]) / 1000, "{:.1f}")
    row("Peak electrical power, full thrust (kW)",
        lambda c: (c.lift_w + c.d["fans"] * c.d["fan_shaft_max"] / COMMON["eta_motor"]
                   + c.d["hotel"]) / 1000, "{:.1f}")
    row("Energy source", lambda c: c.d["power"], "{}")
    row("Endurance at 30 km/h (h)", lambda c: c.endurance_h(TARGETS["cruise"]), "{:.1f}")
    row("Range at 30 km/h (km)", lambda c: c.endurance_h(TARGETS["cruise"]) * 30, "{:.0f}")
    row("Stop from 10 km/h, reverse thrust (m)", lambda c: c.stop_distance(TARGETS["slow"]), "{:.1f}")
    row("Stop from 30 km/h, reverse thrust (m)", lambda c: c.stop_distance(TARGETS["cruise"]), "{:.0f}")
    row("Stop from 50 km/h, reverse thrust (m)", lambda c: c.stop_distance(TARGETS["top"]), "{:.0f}")
    row("Stop from 50 km/h, emergency lift dump (m)",
        lambda c: c.stop_distance(TARGETS["top"], lift_dump=True), "{:.0f}")
    row("Turn radius at 10 / 30 / 50 km/h (m)",
        lambda c: " / ".join(f"{c.turn_radius(v):.0f}" for v in
                             (TARGETS["slow"], TARGETS["cruise"], TARGETS["top"])), "{}")

    lines = ["# Vehicle Version 3 sizing (generated by tools/vehicle_sizing/v3_sizing.py)", "",
             "Stated first-order estimates, not validated data. Version 2 is evaluated with the "
             "same physics for comparison. Note that the Version 2 *simulation* deliberately uses "
             "much higher glide drag, tuned for a ~3 m/s demo top speed.", ""]
    lines += L
    lines += [""]

    # Version 3 checks against the agreed targets
    checks = []
    def check(label, ok, value):
        checks.append(f"- [{'PASS' if ok else 'FAIL'}] {label}: {value}")
    t0 = thrust_available(0.0, v3.d)
    check("Top speed ≥ 50 km/h in calm conditions", v3.top_speed() >= TARGETS["top"],
          f"{v3.top_speed() * 3.6:.0f} km/h")
    check("Cruise 30 km/h into a 5 m/s headwind", v3.top_speed(COMMON["headwind"]) >= TARGETS["cruise"],
          f"top speed into the wind {v3.top_speed(COMMON['headwind']) * 3.6:.0f} km/h")
    check("Static thrust ≥ 15 % of weight (gets over the hump with margin)", t0 / v3.w >= 0.15,
          f"{t0 / v3.w:.2f}")
    check("Cushion pressure within 0.5–1.5 kPa", 500 <= v3.p <= 1500, f"{v3.p / 1000:.2f} kPa")
    check("Genset (20 kW) covers 50 km/h continuously", v3.elec_power(TARGETS["top"]) <= 20000,
          f"{v3.elec_power(TARGETS['top']) / 1000:.1f} kW")
    lines += ["## Version 3 checks against the agreed targets", ""] + checks + [""]

    # Mass budget
    lines += ["## Version 3 mass budget (kg)", "", "| Item | kg |", "|---|---|"]
    lines += [f"| {k} | {v} |" for k, v in v3.d["mass"].items()]
    lines += [f"| **Total** | **{v3.m:.0f}** |", ""]

    # All-electric alternative, same airframe
    alt = dict(v3.d)
    alt["mass"] = {k: v for k, v in v3.d["mass"].items()
                   if k not in ("diesel engine ~20 kW", "generator", "cooling, exhaust, mounts",
                                "fuel 30 L + tank", "buffer battery 5 kWh")}
    alt["mass"]["20 kWh battery"] = 130
    alt.update(power="battery", battery_kwh=20.0, fuel_l=0.0)
    e = Craft("Version 3, all-electric", alt)
    lines += ["## Series hybrid vs all-electric (same airframe)", "",
              "| | Series hybrid (chosen) | All-electric 20 kWh |", "|---|---|---|",
              f"| Mass (kg) | {v3.m:.0f} | {e.m:.0f} |",
              f"| Endurance at 30 km/h (h) | {v3.endurance_h(TARGETS['cruise']):.1f} | "
              f"{e.endurance_h(TARGETS['cruise']):.1f} |",
              f"| Fuel at 30 km/h (L/h) | {v3.fuel_l_h(TARGETS['cruise']):.1f} | – |",
              f"| Top speed, calm (km/h) | {v3.top_speed() * 3.6:.0f} | {e.top_speed() * 3.6:.0f} |", ""]

    # Tracks
    track_w = (COMMON["track_rr"] * v3.w * (1 - COMMON["track_share"]) * COMMON["track_speed"]
               / 0.85 + 0.6 * v3.lift_w + v3.d["hotel"])
    lines += ["## Tracks", "",
              f"Tracks capped at {COMMON['track_speed'] * 3.6:.0f} km/h: about {track_w / 1000:.1f} kW "
              f"electrical with {COMMON['track_share']:.0%} cushion load share. Speed comes from "
              "hovering, not from the tracks.", ""]
    lines += ["## Turning at speed", "",
              "Turn radius = v² / (μ_lat g). μ_lat is the sideways force the craft can make "
              "(skirt side drag, rudders, puff ports) divided by its weight.", "",
              "| μ_lat | 10 km/h | 30 km/h | 50 km/h |", "|---|---|---|---|"]
    for mu, what in ((0.10, "0.10 (rudders + puff ports, as Version 2)"),
                     (0.20, "0.20 (plus skegs / skirt shift)")):
        lines.append(f"| {what} | " + " | ".join(f"{v3.turn_radius(v, mu):.0f} m" for v in
                                                   (TARGETS["slow"], TARGETS["cruise"], TARGETS["top"])) + " |")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", help="also write the report to this Markdown file")
    a = ap.parse_args()
    text = report()
    try:
        sys.stdout.reconfigure(encoding="utf-8")       # Windows consoles default to cp1252
    except AttributeError:
        pass
    print(text)
    if a.md:
        with open(a.md, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)


if __name__ == "__main__":
    main()
