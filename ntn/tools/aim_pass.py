#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Aim a LEO pass over a real place.

leo-link and leo-link-regen-RT place their terminal from the satellite's starting position,
which for a one-satellite shell is a fixed point of the orbit, so no run is ever a pass over a
chosen city. This finds the satellite's starting orbital position for a pass that rises through
--start-elev-deg at t = 0, goes directly overhead the given latitude and longitude, and sets
through the same elevation, then prints the flags that reproduce it:

    ./ntn/tools/aim_pass.py --lat 47.6560453 --lon -122.3094075 --alt-km 500 --incl-deg 53

It mirrors ns3::LeoCircularOrbitMobilityModel exactly - circular orbit, spherical Earth of
radius 6371 km, the orbital plane regressing westward at one turn per 86400 s - and the terminal
sits on the same sphere, as GeocentricConstantPositionMobilityModel places it. The inclination
must be at least the latitude, or no pass reaches the zenith.
"""

import argparse
import math
import sys

R_E = 6371e3  # GeographicPositions::EARTH_SPHERE_RADIUS
GM = 3.986007e14  # GeographicPositions::LEO_EARTH_GGC
DAY = 86400.0  # CalcLongitude's Days(1)


def sat_ecef(t, lon0, u0, alt, incl):
    """LeoCircularOrbitMobilityModel::CalcPosition, term for term."""
    r = R_E + alt
    w = math.sqrt(GM / r**3)
    L = lon0 - t * 2 * math.pi / DAY
    th = u0 + w * t
    x = (math.cos(incl) * math.cos(L), math.cos(incl) * math.sin(L), math.sin(incl))
    kx = (-math.sin(L), math.cos(L), 0.0)  # PlaneNorm x x, simplified: the two are orthogonal
    return tuple(r * (a * math.cos(th) + b * math.sin(th)) for a, b in zip(x, kx))


def elevation(t, obs, lon0, u0, alt, incl):
    s = sat_ecef(t, lon0, u0, alt, incl)
    d = [a - b for a, b in zip(s, obs)]
    up = [c / R_E for c in obs]
    return math.degrees(math.asin(sum(a * b for a, b in zip(d, up)) / math.sqrt(sum(a * a for a in d))))


def overhead_at(T, lat, lon, alt, incl, northbound):
    """Starting (node longitude, argument of latitude) that puts the satellite at the zenith at T."""
    c = math.sin(lat) / math.sin(incl)
    th = math.acos(c) * (-1.0 if northbound else 1.0)  # latitude = asin(sin i cos th)
    L = lon - math.atan2(math.sin(th), math.cos(incl) * math.cos(th))
    w = math.sqrt(GM / (R_E + alt) ** 3)
    return L + T * 2 * math.pi / DAY, th - w * T


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, required=True, help="terminal latitude, degrees")
    ap.add_argument("--lon", type=float, required=True, help="terminal longitude, degrees")
    ap.add_argument("--alt-km", type=float, default=500.0, help="orbit altitude")
    ap.add_argument("--incl-deg", type=float, default=53.0, help="orbit inclination")
    ap.add_argument("--start-elev-deg", type=float, default=10.0, help="elevation at t = 0")
    ap.add_argument("--southbound", action="store_true", help="descending pass instead of ascending")
    a = ap.parse_args()

    lat, lon, incl = map(math.radians, (a.lat, a.lon, a.incl_deg))
    alt = a.alt_km * 1e3
    if abs(math.sin(lat)) > math.sin(incl):
        sys.exit(f"a {a.incl_deg} deg orbit never passes over latitude {a.lat}")
    obs = (R_E * math.cos(lat) * math.cos(lon), R_E * math.cos(lat) * math.sin(lon), R_E * math.sin(lat))

    # Time from the start elevation to the zenith: bisect on T so that elevation(0) = start.
    def start_elev(T):
        lon0, u0 = overhead_at(T, lat, lon, alt, incl, not a.southbound)
        return elevation(0.0, obs, lon0, u0, alt, incl)

    lo, hi = 1.0, 1500.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if start_elev(mid) > a.start_elev_deg else (lo, mid)
    T = 0.5 * (lo + hi)
    lon0, u0 = overhead_at(T, lat, lon, alt, incl, not a.southbound)

    # Where it sets again, and a check of the whole pass.
    t, dt, peak = T, 0.5, 0.0
    while elevation(t, obs, lon0, u0, alt, incl) > a.start_elev_deg:
        peak = max(peak, elevation(t, obs, lon0, u0, alt, incl))
        t += dt
    print(f"# pass: {a.start_elev_deg:g} deg at t=0, zenith at t={T:.1f} s (peak {peak:.2f} deg), "
          f"{a.start_elev_deg:g} deg again at t={t:.1f} s", file=sys.stderr)
    print(f"--observerLatDeg={a.lat} --observerLonDeg={a.lon} "
          f"--satLongitudeDeg={math.degrees(lon0):.6f} --satArgOfLatitudeDeg={math.degrees(u0):.6f} "
          f"--duration={math.ceil(t)}")


if __name__ == "__main__":
    main()
