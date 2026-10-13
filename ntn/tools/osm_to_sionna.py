#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Turn OpenStreetMap building footprints into a Sionna RT scene.

    ./ntn/tools/osm_to_sionna.py ntn/data/uw-campus-buildings.geojson \\
        --origin 47.6560453:-122.3094075 --out ntn/data/uw-campus

Reads a GeoJSON FeatureCollection of footprint polygons with a height_m property (as written for
ntn/data/uw-campus-buildings.geojson) and writes <out>/scene.xml plus two PLY meshes: every
building extruded to its height, and a flat ground square. Sionna 2.1 ships no OSM importer.

Frame: x east, y north, z up, in metres from --origin, on the same 6371 km sphere and the same
flat projection ntn-rt-render.py uses, so the terminal at the origin and the buildings line up
with the renderer. Stand the terminal at x = y = 0 and check it is in the open.

Materials are ITU-R P.2040 as Sionna implements them. Buildings are concrete (valid 1-100 GHz).
The ground is brick, not one of the ITU ground types, because those are only defined up to
10 GHz. Inner courtyards are filled: only each footprint's outer ring is used.

Needs Mitsuba, which Sionna RT installs.
"""

import argparse
import json
import math
import pathlib

R_E = 6371e3


def project(lon, lat, lat0, lon0):
    return (math.radians(lon) - lon0) * R_E * math.cos(lat0), (math.radians(lat) - lat0) * R_E


def signed_area(p):
    return 0.5 * sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(p, p[1:] + p[:1]))


def ear_clip(poly):
    """Triangulate a simple counter-clockwise polygon. Returns index triples."""
    idx = list(range(len(poly)))
    tris = []

    def inside(p, a, b, c):
        def s(u, v, w):
            return (u[0] - w[0]) * (v[1] - w[1]) - (v[0] - w[0]) * (u[1] - w[1])

        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-9:
                continue  # reflex or degenerate corner
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break  # no ear found: leave the rest untriangulated rather than loop forever
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def write_ply(mi, path, verts, faces):
    mesh = mi.Mesh(path.stem, len(verts), len(faces))
    params = mi.traverse(mesh)
    params["vertex_positions"] = [c for v in verts for c in v]
    params["faces"] = [i for f in faces for i in f]
    params.update()
    mesh.write_ply(str(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("geojson")
    ap.add_argument("--origin", required=True, help="scene origin 'lat:lon', degrees")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--ground-m", type=float, default=600.0, help="half-width of the ground square")
    a = ap.parse_args()

    import mitsuba as mi

    mi.set_variant("scalar_rgb")
    lat0, lon0 = (math.radians(float(v)) for v in a.origin.split(":"))
    gj = json.loads(pathlib.Path(a.geojson).read_text())

    verts, faces = [], []
    n_bld = 0
    for feat in gj["features"]:
        ring = [project(lon, lat, lat0, lon0) for lon, lat in feat["geometry"]["coordinates"][0]]
        if ring[0] == ring[-1]:
            ring = ring[:-1]
        # drop repeated points, which break ear clipping
        ring = [p for i, p in enumerate(ring) if math.dist(p, ring[i - 1]) > 1e-3]
        if len(ring) < 3 or abs(signed_area(ring)) < 1.0:
            continue
        if signed_area(ring) < 0:
            ring.reverse()  # counter-clockwise from above: roof normal up, walls outward
        h = float(feat["properties"].get("height_m", 10.0))
        base = len(verts)
        n = len(ring)
        verts += [(x, y, 0.0) for x, y in ring] + [(x, y, h) for x, y in ring]
        for i in range(n):
            j = (i + 1) % n
            faces += [(base + i, base + j, base + n + j), (base + i, base + n + j, base + n + i)]
        faces += [(base + n + i0, base + n + i1, base + n + i2) for i0, i1, i2 in ear_clip(ring)]
        n_bld += 1

    out = pathlib.Path(a.out)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    write_ply(mi, out / "meshes" / "buildings.ply", verts, faces)
    g = a.ground_m
    write_ply(mi, out / "meshes" / "ground.ply",
              [(-g, -g, 0.0), (g, -g, 0.0), (g, g, 0.0), (-g, g, 0.0)], [(0, 1, 2), (0, 2, 3)])
    (out / "scene.xml").write_text(f"""<scene version="2.1.0">
<!-- Generated by ntn/tools/osm_to_sionna.py from {pathlib.Path(a.geojson).name}.
     Origin {a.origin} (lat:lon); x east, y north, z up, metres.
     {gj.get("attribution", "")} -->

	<bsdf type="itu-radio-material" id="concrete">
		<string name="type" value="concrete"/>
		<float name="thickness" value="0.3"/>
	</bsdf>
	<bsdf type="itu-radio-material" id="brick">
		<string name="type" value="brick"/>
		<float name="thickness" value="0.1"/>
	</bsdf>

	<shape type="ply" id="mesh-buildings">
		<string name="filename" value="meshes/buildings.ply"/>
		<ref id="concrete" name="bsdf"/>
	</shape>
	<shape type="ply" id="mesh-ground">
		<string name="filename" value="meshes/ground.ply"/>
		<ref id="brick" name="bsdf"/>
	</shape>
</scene>
""")
    print(f"{n_bld} buildings, {len(verts)} vertices, {len(faces)} triangles -> {out / 'scene.xml'}")


if __name__ == "__main__":
    main()
