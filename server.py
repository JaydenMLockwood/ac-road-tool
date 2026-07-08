#!/usr/bin/env python3
"""
AC Road Tool — Backend server
OSM road data → elevation → FBX with AC markers → Assetto Corsa track zip
"""

import json, math, os, shutil, struct, time, zipfile, io
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from urllib.request import urlopen, Request

import numpy as np
from scipy.interpolate import splprep, splev, interp1d
from scipy.ndimage import uniform_filter1d, gaussian_filter1d
from pyproj import Transformer

PORT = 8743
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ─── OSM Query ────────────────────────────────────────────────────────────────

def search_osm_roads(query: str) -> dict:
    nom_url = (
        f"https://nominatim.openstreetmap.org/search"
        f"?q={query.replace(' ', '+')}&format=json&limit=8&addressdetails=1"
    )
    try:
        req = Request(nom_url, headers={"User-Agent": "AC-Road-Tool/1.0"})
        with urlopen(req, timeout=10) as r:
            results = json.loads(r.read())
    except Exception as e:
        return {"error": str(e), "results": []}

    out = []
    for item in results[:8]:
        out.append({
            "display_name": item.get("display_name", ""),
            "osm_type":     item.get("osm_type", ""),
            "osm_id":       item.get("osm_id", ""),
            "lat":          float(item.get("lat", 0)),
            "lon":          float(item.get("lon", 0)),
            "boundingbox":  item.get("boundingbox", []),
            "type":         item.get("type", "road"),
        })
    return {"results": out}


def _overpass(q: str, timeout: int = 40):
    req = Request(
        "https://overpass-api.de/api/interpreter",
        data=f"data={q}".encode(),
        headers={"User-Agent": "AC-Road-Tool/1.0",
                 "Content-Type": "application/x-www-form-urlencoded"}
    )
    with urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def _stitch_segments(segments: list, seed_idx: int = None,
                     tol_m: float = 40.0) -> tuple:
    """
    Chain road segments (lists of [lat,lon]) into one continuous route by
    matching endpoints. Greedy: start from the seed (or longest) segment,
    repeatedly attach the nearest-endpoint segment to either end, reversing
    as needed, until no segment connects within tol_m.
    Returns (coords, used_count).
    """
    if not segments:
        return [], 0
    segs = [list(s) for s in segments if len(s) >= 2]
    if not segs:
        return [], 0

    def seg_len(s):
        return sum(_haversine(s[i][0], s[i][1], s[i+1][0], s[i+1][1])
                   for i in range(len(s)-1))

    if seed_idx is None or seed_idx >= len(segs):
        seed_idx = max(range(len(segs)), key=lambda i: seg_len(segs[i]))

    chain = segs.pop(seed_idx)
    used = 1

    while segs:
        head, tail = chain[0], chain[-1]
        best = None   # (dist, seg_i, attach_at, reverse)
        for i, s in enumerate(segs):
            for attach_at, cpt in (("tail", tail), ("head", head)):
                for rev, spt in ((False, s[0]), (True, s[-1])) if attach_at == "tail" \
                                 else ((False, s[-1]), (True, s[0])):
                    d = _haversine(cpt[0], cpt[1], spt[0], spt[1])
                    if best is None or d < best[0]:
                        best = (d, i, attach_at, rev)
        if best is None or best[0] > tol_m:
            break
        _, i, attach_at, rev = best
        s = segs.pop(i)
        if rev:
            s = list(reversed(s))
        if attach_at == "tail":
            # drop duplicated joint point if coincident
            if _haversine(chain[-1][0], chain[-1][1], s[0][0], s[0][1]) < 1.0:
                s = s[1:]
            chain.extend(s)
        else:
            if _haversine(chain[0][0], chain[0][1], s[-1][0], s[-1][1]) < 1.0:
                s = s[:-1]
            chain = s + chain
        used += 1

    return chain, used


def fetch_road_geometry(osm_type: str, osm_id: str) -> dict:
    """
    Fetch the FULL road, not just the selected OSM way. Roads are split
    into many ways in OSM; we read the selected way's name/ref, gather all
    same-named ways in the surrounding area, and stitch them into one
    continuous route.
    """
    try:
        if osm_type == "way":
            data = _overpass(f"[out:json];way({osm_id});out geom;")
        elif osm_type == "relation":
            data = _overpass(f"[out:json];relation({osm_id});way(r);out geom;")
        else:
            return {"error": "Unsupported OSM type", "coords": []}
    except Exception as e:
        return {"error": str(e), "coords": []}

    elements = [el for el in data.get("elements", [])
                if el.get("type") == "way" and "geometry" in el]
    if not elements:
        return {"error": "No geometry found", "coords": []}

    segments = [[[g["lat"], g["lon"]] for g in el["geometry"]]
                for el in elements]
    seed_idx = 0

    # ── Name expansion (single way only — relations already give members)
    if osm_type == "way":
        tags = elements[0].get("tags", {})
        name = tags.get("name") or tags.get("ref")
        key  = "name" if tags.get("name") else "ref"
        if name:
            # bbox of the selected way, padded ~0.25° (~25 km)
            lats = [g["lat"] for g in elements[0]["geometry"]]
            lons = [g["lon"] for g in elements[0]["geometry"]]
            pad = 0.25
            bbox = (min(lats)-pad, min(lons)-pad, max(lats)+pad, max(lons)+pad)
            esc = name.replace('\\', '\\\\').replace('"', '\\"')
            q = (f"[out:json][timeout:30];"
                 f"way[\"{key}\"=\"{esc}\"][\"highway\"]"
                 f"({bbox[0]:.4f},{bbox[1]:.4f},{bbox[2]:.4f},{bbox[3]:.4f});"
                 f"out geom;")
            try:
                more = _overpass(q)
                more_els = [el for el in more.get("elements", [])
                            if el.get("type") == "way" and "geometry" in el]
                if len(more_els) > len(elements):
                    seed_geom = segments[0]
                    segments = [[[g["lat"], g["lon"]] for g in el["geometry"]]
                                for el in more_els]
                    # seed = the originally selected way (match by first point)
                    seed_idx = 0
                    for i, s in enumerate(segments):
                        if s[0] == seed_geom[0] and s[-1] == seed_geom[-1]:
                            seed_idx = i
                            break
                    print(f"  [geometry] '{name}': expanded from 1 to "
                          f"{len(segments)} way segments")
            except Exception as e:
                print(f"  [geometry] name expansion failed ({e}) — "
                      f"using selected way only")

    coords, used = _stitch_segments(segments, seed_idx)
    if len(coords) < 2:
        return {"error": "No geometry found", "coords": []}
    print(f"  [geometry] stitched {used}/{len(segments)} segments, "
          f"{len(coords)} points")
    return {"coords": coords, "count": len(coords),
            "segments_used": used, "segments_total": len(segments)}


def _haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((lat2-lat1)*math.pi/360)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin((lon2-lon1)*math.pi/360)**2)
    return 2 * R * math.asin(math.sqrt(a))


# ─── Surroundings (buildings + trees from OSM, free via Overpass) ─────────────

def fetch_surroundings(coords: list, radius_m: float = 60.0) -> dict:
    """
    Fetch buildings and trees within radius_m of the route from Overpass.
    Returns {"buildings": [{"outline": [[lat,lon],...], "height": m}],
             "trees": [[lat, lon], ...]}  (empty lists on failure).
    """
    out = {"buildings": [], "trees": [], "forests": []}
    if len(coords) < 2:
        return out

    # Overpass 'around' accepts a polyline — downsample to ≤120 points
    step = max(1, len(coords) // 120)
    line_pts = coords[::step]
    if line_pts[-1] != coords[-1]:
        line_pts.append(coords[-1])
    line = ",".join(f"{c[0]:.6f},{c[1]:.6f}" for c in line_pts)

    q = (f"[out:json][timeout:40];"
         f"(way[\"building\"](around:{radius_m:.0f},{line});"
         f"node[\"natural\"=\"tree\"](around:{radius_m:.0f},{line});"
         f"way[\"natural\"=\"tree_row\"](around:{radius_m:.0f},{line});"
         f"way[\"natural\"~\"wood|scrub|heath\"](around:{radius_m:.0f},{line});"
         f"way[\"landuse\"~\"forest|orchard\"](around:{radius_m:.0f},{line});"
         f"relation[\"natural\"=\"wood\"](around:{radius_m:.0f},{line});"
         f"relation[\"landuse\"=\"forest\"](around:{radius_m:.0f},{line});"
         f"way[\"leisure\"=\"nature_reserve\"](around:{radius_m:.0f},{line});"
         f"relation[\"leisure\"=\"nature_reserve\"](around:{radius_m:.0f},{line});"
         f"relation[\"boundary\"~\"national_park|protected_area\"]"
         f"(around:{radius_m:.0f},{line}););"
         f"out geom 4000;")
    try:
        data = _overpass(q, timeout=50)
    except Exception as e:
        print(f"  [surroundings] Overpass failed: {e}")
        return out

    def veg_density(tags):
        """1.0 = dense woodland, lower = sparser scatter."""
        if tags.get("natural") in ("wood",) or tags.get("landuse") == "forest":
            return 1.0
        if tags.get("natural") in ("scrub", "heath") or tags.get("landuse") == "orchard":
            return 0.5
        if tags.get("leisure") == "nature_reserve" or "boundary" in tags:
            return 0.35     # parks/reserves: sparse bush scatter
        return 0.0

    seen = set()

    def parse_elements(elements):
        for el in elements:
            eid = (el.get("type"), el.get("id"))
            if eid in seen:
                continue
            seen.add(eid)
            tags = el.get("tags", {})
            if el.get("type") == "way" and "geometry" in el:
                outline = [[g["lat"], g["lon"]] for g in el["geometry"] if g]
                if "building" in tags:
                    # height: explicit metres > levels×3 > default 5m
                    h = 5.0
                    try:
                        if "height" in tags:
                            h = float(str(tags["height"]).replace("m", "").strip())
                        elif "building:levels" in tags:
                            h = float(tags["building:levels"]) * 3.0
                    except ValueError:
                        pass
                    h = max(2.5, min(h, 60.0))
                    if len(outline) >= 3:
                        out["buildings"].append({"outline": outline, "height": h})
                elif tags.get("natural") == "tree_row":
                    # plant a tree every ~8m along the row
                    for j in range(len(outline) - 1):
                        a, b = outline[j], outline[j+1]
                        d = _haversine(a[0], a[1], b[0], b[1])
                        n = max(1, int(d / 8))
                        for k in range(n):
                            t = k / n
                            out["trees"].append([a[0] + (b[0]-a[0])*t,
                                                 a[1] + (b[1]-a[1])*t])
                else:
                    dns = veg_density(tags)
                    if dns > 0 and len(outline) >= 3:
                        out["forests"].append({"outline": outline,
                                               "density": dns})
            elif el.get("type") == "relation" and "members" in el:
                # multipolygon vegetation (national parks, big forests) —
                # each outer member ring becomes a scatter polygon
                dns = veg_density(tags)
                if dns > 0:
                    for m in el["members"]:
                        if m.get("type") == "way" and "geometry" in m \
                           and m.get("role") in ("outer", ""):
                            outline = [[g["lat"], g["lon"]]
                                       for g in m["geometry"] if g]
                            if len(outline) >= 3:
                                out["forests"].append({"outline": outline,
                                                       "density": dns})
            elif el.get("type") == "node" and "lat" in el:
                out["trees"].append([el["lat"], el["lon"]])

    parse_elements(data.get("elements", []))

    # ── Containment pass ──
    # `around` only measures distance to a polygon's OUTLINE, so a road
    # running through the middle of a big park/forest (boundary far away)
    # matches nothing. Sample points on the road and ask which vegetation
    # areas CONTAIN them.
    n_samp = 6
    samp = [coords[int(i * (len(coords)-1) / (n_samp-1))] for i in range(n_samp)]
    isin = "".join(f"is_in({c[0]:.6f},{c[1]:.6f});" for c in samp)
    # clip returned geometry to the road's bbox (padded ~2km) so a giant
    # national-park polygon doesn't return megabytes
    lats = [c[0] for c in coords]; lons = [c[1] for c in coords]
    clip = (f"{min(lats)-0.02:.4f},{min(lons)-0.02:.4f},"
            f"{max(lats)+0.02:.4f},{max(lons)+0.02:.4f}")
    q2 = (f"[out:json][timeout:30];({isin})->.a;"
          f"(way(pivot.a)[\"natural\"~\"wood|scrub|heath\"];"
          f"way(pivot.a)[\"landuse\"~\"forest|orchard\"];"
          f"way(pivot.a)[\"leisure\"=\"nature_reserve\"];"
          f"rel(pivot.a)[\"natural\"=\"wood\"];"
          f"rel(pivot.a)[\"landuse\"=\"forest\"];"
          f"rel(pivot.a)[\"leisure\"=\"nature_reserve\"];"
          f"rel(pivot.a)[\"boundary\"~\"national_park|protected_area\"];);"
          f"out geom({clip}) 500;")
    try:
        time.sleep(1.0)               # be polite to Overpass between calls
        data2 = _overpass(q2, timeout=40)
        parse_elements(data2.get("elements", []))
    except Exception as e:
        print(f"  [surroundings] containment query failed: {e}")

    # Caps to keep the KN5 sane
    out["buildings"] = out["buildings"][:800]
    out["trees"]     = out["trees"][:1500]
    out["forests"]   = out["forests"][:300]
    dens = [f["density"] for f in out["forests"]]
    print(f"  [surroundings] {len(out['buildings'])} buildings, "
          f"{len(out['trees'])} tree nodes, {len(out['forests'])} vegetation "
          f"areas (max density {max(dens) if dens else 0})")
    return out


def build_environment_meshes(surroundings: dict, mesh: dict,
                             ground_pts: list = None) -> dict:
    """
    Convert OSM buildings/trees into KN5 mesh tuples in the track's local
    coordinate space. Returns [(name, kn5_verts, indices, material_key)].
    Visual only — names carry no digit prefix so there is no physics.
    """
    if not surroundings:
        return []
    proj_p = mesh["proj"]
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={proj_p['mid_lat']} +lon_0={proj_p['mid_lon']} +units=m",
        always_xy=True)
    ox, oz = proj_p["ox"], proj_p["oz"]

    # Ground height lookup. With real terrain: nearest terrain/road vertex.
    # Without: nearest centerline point + synthetic lateral drop.
    from scipy.spatial import cKDTree
    cl = mesh["centerline"]
    hw = mesh["stats"]["road_width"] / 2.0
    if ground_pts:
        g_xz = np.array([[p[0], p[2]] for p in ground_pts])
        g_y  = np.array([p[1] for p in ground_pts])
        kd = cKDTree(g_xz)

        def ground_y(x, z):
            _, i = kd.query([x, z])
            return float(g_y[i])
    else:
        cl_xz = np.array([[p[0], p[2]] for p in cl])
        cl_y  = np.array([p[1] for p in cl])
        kd = cKDTree(cl_xz)

        def ground_y(x, z):
            d, i = kd.query([x, z])
            return float(cl_y[i]) - ground_drop(float(d) - hw)

    def to_local(lat, lon):
        X, Z = proj.transform(lon, lat)
        return X - ox, -Z - oz     # -Z: same handedness flip as the road

    # ── Scatter trees inside vegetation polygons ──
    # OSM forests are areas, not tree nodes — walk the road, throw random
    # points into the corridor, keep ones that land inside vegetation,
    # accepting with the area's density (woods dense, reserves sparse).
    tree_pts = [to_local(la, lo) for la, lo in surroundings.get("trees", [])]

    forests = []
    for f in surroundings.get("forests", []):
        outline = f["outline"] if isinstance(f, dict) else f
        density = f.get("density", 1.0) if isinstance(f, dict) else 1.0
        poly = [to_local(la, lo) for la, lo in outline[::max(1, len(outline)//200)]]
        if len(poly) >= 3:
            pxs = [p[0] for p in poly]; pzs = [p[1] for p in poly]
            forests.append((poly, (min(pxs), min(pzs), max(pxs), max(pzs)),
                            density))

    def in_poly(x, z, poly):
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, zi = poly[i]; xj, zj = poly[j]
            if (zi > z) != (zj > z) and \
               x < (xj - xi) * (z - zi) / ((zj - zi) or 1e-12) + xi:
                inside = not inside
            j = i
        return inside

    if forests and len(tree_pts) < 1500:
        rng = np.random.default_rng(99)
        min_lat = hw + 4.0          # keep off road and verge
        max_lat = hw + GRASS_W + SKIRT_W - 3.0
        step = 12                    # try planting every 12m of road
        for ci in range(0, len(cl), step):
            if len(tree_pts) >= 1500:
                break
            p = cl[ci]
            pn = cl[min(ci + 1, len(cl) - 1)]
            dx, dz = pn[0] - p[0], pn[2] - p[2]
            L = math.hypot(dx, dz) or 1.0
            px_, pz_ = -dz / L, dx / L     # perpendicular
            for side in (+1, -1):
                for _ in range(2):
                    lat_d = rng.uniform(min_lat, max_lat)
                    along = rng.uniform(-6, 6)
                    x = p[0] + side * px_ * lat_d + dx / L * along
                    z = p[2] + side * pz_ * lat_d + dz / L * along
                    for poly, bb, density in forests:
                        if bb[0] <= x <= bb[2] and bb[1] <= z <= bb[3] \
                           and rng.random() < density \
                           and in_poly(x, z, poly):
                            tree_pts.append((x, z))
                            break

    meshes = []

    # ── Buildings: extruded footprints ──
    verts, idx = [], []
    part = 0

    def flush_buildings():
        nonlocal verts, idx, part
        if verts:
            meshes.append((f"ENV_BLDG_{part}", verts, idx, "building"))
            part += 1
            verts, idx = [], []

    for b in surroundings.get("buildings", []):
        pts = [to_local(la, lo) for la, lo in b["outline"]]
        if len(pts) > 2 and pts[0] == pts[-1]:
            pts = pts[:-1]
        if len(pts) < 3 or len(pts) > 60:
            continue
        base = min(ground_y(x, z) for x, z in pts) - 0.5
        top  = base + 0.5 + b["height"]

        if len(verts) + len(pts)*5 > 60000:
            flush_buildings()
        # Walls: quad per edge, double-sided (interior winding unknown)
        for i in range(len(pts)):
            x0, z0 = pts[i]
            x1, z1 = pts[(i+1) % len(pts)]
            ex, ez = x1-x0, z1-z0
            el = math.hypot(ex, ez) or 1.0
            nx_, nz_ = -ez/el, ex/el
            u1 = el / 4.0
            v = len(verts)
            verts.append(((x0, base, z0), (nx_, 0, nz_), (0,  0), (ex/el, 0, ez/el)))
            verts.append(((x1, base, z1), (nx_, 0, nz_), (u1, 0), (ex/el, 0, ez/el)))
            verts.append(((x1, top,  z1), (nx_, 0, nz_), (u1, 1), (ex/el, 0, ez/el)))
            verts.append(((x0, top,  z0), (nx_, 0, nz_), (0,  1), (ex/el, 0, ez/el)))
            idx.extend((v, v+1, v+2,  v, v+2, v+3))
            idx.extend((v, v+2, v+1,  v, v+3, v+2))

        # Roof: triangle fan (fine for mostly-convex footprints), double-sided
        r0 = len(verts)
        for x, z in pts:
            verts.append(((x, top, z), (0, 1, 0), (x*0.1, z*0.1), (1, 0, 0)))
        for i in range(1, len(pts)-1):
            idx.extend((r0, r0+i, r0+i+1))
            idx.extend((r0, r0+i+1, r0+i))
    flush_buildings()

    # ── Trees: 4-sided trunk + pyramid canopy, double-sided ──
    verts, idx = [], []
    part = 0

    def flush_trees():
        nonlocal verts, idx, part
        if verts:
            meshes.append((f"ENV_TREE_{part}", verts, idx, "tree"))
            part += 1
            verts, idx = [], []

    rng_t = np.random.default_rng(7)
    for x, z in tree_pts:
        gy = ground_y(x, z)
        s = rng_t.uniform(0.7, 1.4)                     # size variety
        th, ch, cr, tr = 2.0*s, 5.0*s, 2.2*s, 0.25*s   # trunk h, canopy h/r, trunk r

        if len(verts) + 13 > 60000:
            flush_trees()
        v = len(verts)
        corners = [(-tr,-tr),(tr,-tr),(tr,tr),(-tr,tr)]
        for cx, cz in corners:
            verts.append(((x+cx, gy,    z+cz), (cx/tr*0.7, 0, cz/tr*0.7), (0,0), (1,0,0)))
            verts.append(((x+cx, gy+th, z+cz), (cx/tr*0.7, 0, cz/tr*0.7), (0,1), (1,0,0)))
        for i in range(4):
            a = v + i*2; b = v + ((i+1) % 4)*2
            idx.extend((a, b, b+1,  a, b+1, a+1))
            idx.extend((a, b+1, b,  a, a+1, b+1))
        c = len(verts)
        for cx, cz in [(-cr,-cr),(cr,-cr),(cr,cr),(-cr,cr)]:
            verts.append(((x+cx, gy+th, z+cz), (cx/cr*0.6, 0.5, cz/cr*0.6), (0,0), (1,0,0)))
        verts.append(((x, gy+th+ch, z), (0, 1, 0), (0.5, 1), (1, 0, 0)))
        apex = c + 4
        for i in range(4):
            a = c + i; b = c + (i+1) % 4
            idx.extend((a, b, apex))
            idx.extend((a, apex, b))
        idx.extend((c, c+2, c+1,  c, c+3, c+2))
        idx.extend((c, c+1, c+2,  c, c+2, c+3))
    flush_trees()

    return {"meshes": meshes, "n_trees": len(tree_pts),
            "n_buildings": len(surroundings.get("buildings", []))}


# ─── Elevation ────────────────────────────────────────────────────────────────

# Disk cache: repeated exports of the same road cost zero API calls.
_ELEV_CACHE_PATH = os.path.join(OUTPUT_DIR, "elev_cache.json")
try:
    with open(_ELEV_CACHE_PATH) as _f:
        _elev_cache = json.load(_f)
except Exception:
    _elev_cache = {}


def _ck(c):
    return f"{c[0]:.5f},{c[1]:.5f}"      # ~1m precision


def _save_elev_cache():
    try:
        with open(_ELEV_CACHE_PATH, 'w') as f:
            json.dump(_elev_cache, f)
    except Exception:
        pass


def fetch_elevations(coords: list, prefer_openmeteo: bool = False):
    """
    Cached, rate-limit-friendly elevation lookup. Serves hits from the
    disk cache and only queries the APIs for misses. Large batches
    (terrain grids) should prefer open-meteo, whose free limits are far
    higher (600/min, 10k/day) than opentopodata's (1/sec, 1000/day).
    Returns list of floats or None.
    """
    missing = [c for c in coords if _ck(c) not in _elev_cache]
    if missing:
        n_hit = len(coords) - len(missing)
        if n_hit:
            print(f"  [elevation] {n_hit}/{len(coords)} from cache, "
                  f"fetching {len(missing)}")
        order = ([_fetch_openmeteo, _fetch_opentopodata] if prefer_openmeteo
                 else [_fetch_opentopodata, _fetch_openmeteo])
        got = order[0](missing)
        if got is None:
            got = order[1](missing)
        if got is None:
            return None
        for c, e in zip(missing, got):
            _elev_cache[_ck(c)] = float(e or 0)
        _save_elev_cache()
    return [_elev_cache[_ck(c)] for c in coords]

def fetch_elevation_profile(coords: list) -> dict:
    """
    Sample elevation at UNIFORM DISTANCE intervals along the route
    (not at the raw GPS nodes, which are unevenly spaced — dense in
    bends, sparse on straights — and would distort gradients).

    Returns {"dists": [m along route], "elevs": [m]} or None.
    """
    if len(coords) < 2:
        return None

    # Arc length along the raw polyline (haversine, metres)
    dists = [0.0]
    for i in range(1, len(coords)):
        dists.append(dists[-1] + _haversine(coords[i-1][0], coords[i-1][1],
                                            coords[i][0],   coords[i][1]))
    total = dists[-1]

    # Uniform samples: every 20 m, capped at 500 API points.
    # SRTM is ~30 m resolution, so denser sampling adds nothing.
    interval = max(20.0, total / 499.0)
    n_samples = max(int(total / interval) + 1, 4)
    sample_d = [min(i * interval, total) for i in range(n_samples)]

    lat_f = interp1d(dists, [c[0] for c in coords])
    lon_f = interp1d(dists, [c[1] for c in coords])
    sample_coords = [[float(lat_f(d)), float(lon_f(d))] for d in sample_d]

    elevs = fetch_elevations(sample_coords)   # small batch: SRTM 30m first
    if elevs is None:
        print("  [elevation] APIs unavailable — using flat terrain")
        return None

    return {"dists": sample_d, "elevs": elevs}


def fetch_elevation(coords: list) -> list:
    """Legacy per-coordinate elevation (kept for API compatibility)."""
    prof = fetch_elevation_profile(coords)
    if prof is None:
        return [0.0] * len(coords)
    dists = [0.0]
    for i in range(1, len(coords)):
        dists.append(dists[-1] + _haversine(coords[i-1][0], coords[i-1][1],
                                            coords[i][0],   coords[i][1]))
    f = interp1d(prof["dists"], prof["elevs"], kind='linear',
                 fill_value='extrapolate')
    return [float(f(d)) for d in dists]


def _fetch_opentopodata(coords):
    # Public limits: 100 locations/call, 1 call/second, 1000 calls/day.
    BATCH = 100
    results = []
    for i in range(0, len(coords), BATCH):
        batch = coords[i:i+BATCH]
        loc = "|".join(f"{c[0]},{c[1]}" for c in batch)
        for attempt in (1, 2):
            try:
                req = Request(f"https://api.opentopodata.org/v1/srtm30m?locations={loc}",
                              headers={"User-Agent": "AC-Road-Tool/1.0"})
                with urlopen(req, timeout=15) as r:
                    data = json.loads(r.read())
                if data.get("status") != "OK":
                    return None
                results.extend(float(p["elevation"] or 0) for p in data["results"])
                break
            except Exception as e:
                if "429" in str(e) and attempt == 1:
                    print(f"  [opentopodata] rate limited — backing off 5s…")
                    time.sleep(5.0)
                    continue
                print(f"  [opentopodata] {e}")
                return None
        if i + BATCH < len(coords):
            time.sleep(1.05)          # respect the 1 call/second limit
    return results if len(results) == len(coords) else None


def _fetch_openmeteo(coords):
    # Free limits: ~600 calls/min, 10k/day — the roomier option for
    # large terrain batches.
    BATCH = 100
    results = []
    for i in range(0, len(coords), BATCH):
        batch = coords[i:i+BATCH]
        lats = ",".join(str(c[0]) for c in batch)
        lons = ",".join(str(c[1]) for c in batch)
        for attempt in (1, 2):
            try:
                req = Request(
                    f"https://api.open-meteo.com/v1/elevation?latitude={lats}&longitude={lons}",
                    headers={"User-Agent": "AC-Road-Tool/1.0"})
                with urlopen(req, timeout=15) as r:
                    data = json.loads(r.read())
                elevs = data.get("elevation", [])
                if not elevs:
                    return None
                results.extend(float(e) for e in elevs)
                break
            except Exception as e:
                if "429" in str(e) and attempt == 1:
                    print(f"  [open-meteo] rate limited — backing off 5s…")
                    time.sleep(5.0)
                    continue
                print(f"  [open-meteo] {e}")
                return None
        if i + BATCH < len(coords):
            time.sleep(0.2)
    return results if len(results) == len(coords) else None


# ─── Road Geometry ────────────────────────────────────────────────────────────

# Lateral terrain profile (metres from road edge → height drop below road)
GRASS_W    = 10.0   # grass verge width each side
GRASS_DROP = 0.4    # drop at grass outer edge
SKIRT_W    = 70.0   # terrain skirt beyond the grass (10m → 80m out)
SKIRT_DROP = 2.0    # total drop at skirt outer edge


def ground_drop(lateral_from_road_edge: float) -> float:
    """Height drop of the terrain at a lateral distance from the road EDGE."""
    d = max(0.0, lateral_from_road_edge)
    if d <= GRASS_W:
        return GRASS_DROP * (d / GRASS_W)
    if d <= GRASS_W + SKIRT_W:
        return GRASS_DROP + (SKIRT_DROP - GRASS_DROP) * ((d - GRASS_W) / SKIRT_W)
    return SKIRT_DROP


# ─── Real Terrain (lateral elevation grid) ────────────────────────────────────

def fetch_terrain_grid(mesh: dict, max_stations: int = 250) -> dict:
    """
    Sample REAL elevation on a lateral grid beside the road: at stations
    every ~15m+ along the centerline, query elevation at several offsets
    each side. Cliffs, valleys and hillsides beside the road come from
    real data. Returns {"stations": [cl indices], "rings": [m from
    centerline], "deltas": (S, 2, R) array of height vs road} or None.
    """
    cl = mesh["centerline"]
    proj_p = mesh["proj"]
    hw = mesh["stats"]["road_width"] / 2.0
    g0 = hw + GRASS_W
    rings = [g0 + 10, g0 + 32, g0 + 65]             # metres from centerline

    step = max(20, int(len(cl) / max_stations))
    stations = list(range(0, len(cl), step))
    if stations[-1] != len(cl) - 1:
        stations.append(len(cl) - 1)
    S, R = len(stations), len(rings)

    inv = Transformer.from_crs(
        f"+proj=tmerc +lat_0={proj_p['mid_lat']} +lon_0={proj_p['mid_lon']} +units=m",
        "EPSG:4326", always_xy=True)
    ox, oz = proj_p["ox"], proj_p["oz"]

    def to_latlon(x, z):
        lon, lat = inv.transform(x + ox, -(z + oz))   # undo handedness flip
        return [lat, lon]

    # Build query list: per station — centre + R offsets each side
    query = []
    for si in stations:
        p  = cl[si]
        pn = cl[min(si + 1, len(cl) - 1)]
        dx, dz = pn[0] - p[0], pn[2] - p[2]
        L = math.hypot(dx, dz) or 1.0
        px_, pz_ = -dz / L, dx / L
        query.append(to_latlon(p[0], p[2]))
        for side in (+1, -1):
            for r in rings:
                query.append(to_latlon(p[0] + side * px_ * r,
                                       p[2] + side * pz_ * r))

    print(f"  [terrain] sampling {len(query)} elevation points "
          f"({S} stations × {2*R+1})…")
    elevs = fetch_elevations(query, prefer_openmeteo=True)  # big batch: roomier limits
    if elevs is None:
        print("  [terrain] elevation APIs unavailable — flat skirt fallback")
        return None

    per = 2 * R + 1
    deltas = np.zeros((S, 2, R))
    for i in range(S):
        base = elevs[i * per]
        for s in range(2):                     # 0 = left(+), 1 = right(-)
            for r in range(R):
                e = elevs[i * per + 1 + s * R + r]
                d = (e - base) if (e is not None and base is not None) else 0.0
                deltas[i, s, r] = max(-120.0, min(120.0, d))
    return {"stations": stations, "rings": rings, "deltas": deltas}


def build_terrain_meshes(mesh: dict, tgrid: dict) -> list:
    """
    Mesh the real-elevation lateral grid into two physical terrain strips
    (1GRASS_TL / 1GRASS_TR). Returns [(name, kn5_verts, indices, mat_key)].
    """
    cl = mesh["centerline"]
    hw = mesh["stats"]["road_width"] / 2.0
    g0 = hw + GRASS_W
    stations = tgrid["stations"]
    rings    = tgrid["rings"]
    deltas   = tgrid["deltas"]
    S, R = len(stations), len(rings)
    ring_d = [g0] + list(rings)                 # ring 0 = grass outer edge

    out = []
    for s_i, side in ((0, +1), (1, -1)):
        # vertex grid: rows = stations, cols = rings (grass edge → outermost)
        grid = []                                # (S, R+1) of (x, y, z)
        for i, ci in enumerate(stations):
            p  = cl[ci]
            pn = cl[min(ci + 1, len(cl) - 1)]
            dx, dz = pn[0] - p[0], pn[2] - p[2]
            L = math.hypot(dx, dz) or 1.0
            px_, pz_ = -dz / L, dx / L
            row = []
            for r, d in enumerate(ring_d):
                x = p[0] + side * px_ * d
                z = p[2] + side * pz_ * d
                if r == 0:
                    y = p[1] - GRASS_DROP
                else:
                    y = p[1] + float(deltas[i, s_i, r - 1])
                row.append((x, y, z))
            grid.append(row)

        # verts with normals from grid neighbours
        kv = []
        C = R + 1
        for i in range(S):
            for r in range(C):
                x, y, z = grid[i][r]
                i2 = min(i + 1, S - 1); i1 = max(i - 1, 0)
                r2 = min(r + 1, C - 1); r1 = max(r - 1, 0)
                a = np.array(grid[i2][r]) - np.array(grid[i1][r])
                b = np.array(grid[i][r2]) - np.array(grid[i][r1])
                n = np.cross(a, b)
                ln = np.linalg.norm(n) or 1.0
                n = n / ln
                if n[1] < 0:
                    n = -n
                uv = (r / R, (stations[i] / max(1, len(cl))) * 40.0)
                kv.append(((x, y, z), tuple(n), uv, (1.0, 0.0, 0.0)))

        idx = []
        for i in range(S - 1):
            for r in range(C - 1):
                a = i * C + r
                b = a + 1
                c = a + C
                d = c + 1
                # geometric up-winding check on first quad orientation
                idx.extend((a, d, b)); idx.extend((a, c, d))
        # fix winding: test one quad's geometric normal
        p0 = np.array(kv[0][0]); p1 = np.array(kv[1][0]); pc = np.array(kv[C][0])
        gy = np.cross(pc - p0, p1 - p0)[1]
        if gy < 0:
            fixed = []
            for j in range(0, len(idx), 3):
                fixed.extend((idx[j], idx[j+2], idx[j+1]))
            idx = fixed

        name = "1GRASS_TL" if side > 0 else "1GRASS_TR"
        # chunk if oversized (rare)
        if len(kv) <= 60000:
            out.append((name, kv, idx, "terrain"))
        else:
            rows_per = 60000 // C
            part = 0
            r0 = 0
            while r0 < S - 1:
                r1 = min(r0 + rows_per, S)
                sub_kv = kv[r0*C:r1*C]
                sub_idx = []
                for i in range(r1 - r0 - 1):
                    for r in range(C - 1):
                        a = i * C + r
                        sub_idx.extend((a, a+C+1, a+1, a, a+C, a+C+1))
                out.append((f"{name}_{part}", sub_kv, sub_idx, "terrain"))
                r0 = r1 - 1
                part += 1
    return out

def process_road(coords: list, road_width: float = 8.0,
                 smooth_factor: float = 0.3, elevations: list = None,
                 elev_profile: dict = None, grass_width: float = 10.0) -> dict:
    if len(coords) < 4:
        return {"error": "Need at least 4 points"}

    mid_lat = np.mean([c[0] for c in coords])
    mid_lon = np.mean([c[1] for c in coords])
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={mid_lat} +lon_0={mid_lon} +units=m",
        always_xy=True
    )
    pts = np.array([proj.transform(c[1], c[0]) for c in coords])
    # AC uses a left-handed world (north = -Z). Without this negation the
    # whole track is mirrored: left turns become right turns.
    pts[:, 1] = -pts[:, 1]

    # Resample to 1m spacing
    diffs = np.diff(pts, axis=0)
    seg_len = np.sqrt((diffs**2).sum(axis=1))
    arc = np.concatenate([[0], np.cumsum(seg_len)])
    total_length = arc[-1]
    n_pts = max(int(total_length), 4)
    t_u = np.linspace(0, arc[-1], n_pts)
    x_rs = np.interp(t_u, arc, pts[:, 0])
    z_rs = np.interp(t_u, arc, pts[:, 1])

    # ── Elevation ──
    # Preferred path: distance-based profile from fetch_elevation_profile.
    # Interpolating by DISTANCE (not GPS point index) keeps gradients true.
    if elev_profile and elev_profile.get("elevs"):
        prof_d = np.array(elev_profile["dists"], dtype=float)
        prof_e = np.array(elev_profile["elevs"], dtype=float)
        # profile distances are haversine along raw coords; rescale to
        # match projected arc length (they differ by <0.1%)
        if prof_d[-1] > 0:
            prof_d = prof_d * (total_length / prof_d[-1])
        y_rs = np.interp(t_u, prof_d, prof_e)
        y_rs -= y_rs.min()
    elif elevations and len(elevations) == len(coords):
        raw_elev = np.array(elevations, dtype=float)
        raw_elev -= raw_elev.min()
        y_rs = np.interp(t_u, arc, raw_elev)
    else:
        y_rs = np.zeros(n_pts)

    # Smooth XZ planform
    smoothing = smooth_factor * n_pts
    try:
        tck, _ = splprep([x_rs, z_rs], s=smoothing, per=False, k=3)
        xs, zs = splev(np.linspace(0, 1, n_pts), tck)
    except Exception:
        xs, zs = x_rs, z_rs

    # ── Smooth elevation ──
    # SRTM data is ~30m resolution with ±1-2m noise, so a Gaussian with
    # sigma ≈ 30m removes quantisation steps without flattening real
    # gradients (nothing under 30m is real data anyway). Gaussian (not
    # boxcar) => continuous slope => no felt "kinks" at speed.
    sigma_m = 30.0
    ys = np.array(gaussian_filter1d(y_rs, sigma=sigma_m, mode='nearest'))

    # Centre at road start (origin = road start)
    ox, oz = float(xs[0]), float(zs[0])
    xs = xs - ox
    zs = zs - oz

    # Road edges
    dxs = np.gradient(xs)
    dzs = np.gradient(zs)
    lengths_xz = np.sqrt(dxs**2 + dzs**2) + 1e-9
    nx = -dzs / lengths_xz
    nz =  dxs / lengths_xz
    hw = road_width / 2.0
    gw = GRASS_W
    g_drop = GRASS_DROP
    lx = xs + nx*hw;  lz = zs + nz*hw
    rx = xs - nx*hw;  rz = zs - nz*hw
    # Grass outer edges
    glx = xs + nx*(hw+gw);  glz = zs + nz*(hw+gw)  # left grass outer
    grx = xs - nx*(hw+gw);  grz = zs - nz*(hw+gw)  # right grass outer
    # Terrain skirt outer edges (grass outer → 80m from road)
    sw = gw + SKIRT_W
    slx_ = xs + nx*(hw+sw);  slz_ = zs + nz*(hw+sw)
    srx_ = xs - nx*(hw+sw);  srz_ = zs - nz*(hw+sw)

    # Build road mesh
    vertices, uvs, faces = [], [], []
    ts = 0.1  # texture scale
    for i in range(n_pts):
        uc = i / (n_pts-1) * total_length * ts
        vertices.append((lx[i], float(ys[i]), lz[i]))
        vertices.append((rx[i], float(ys[i]), rz[i]))
        uvs.append((0.0, uc)); uvs.append((1.0, uc))
    for i in range(n_pts - 1):
        a,b,c,d = i*2, i*2+1, i*2+2, i*2+3
        # Winding: counter-clockwise when viewed from above = normals point up
        faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
        faces.append((a+1, c+1, d+1, a+1, c+1, d+1))

    # Build grass vertices (left and right strips)
    grass_l_verts = []
    grass_r_verts = []
    grass_uvs = []
    grass_l_faces = []
    grass_r_faces = []

    for i in range(n_pts):
        uc = i / (n_pts-1) * total_length * ts
        oy = float(ys[i]) - g_drop          # outer edge blends downward
        # Left grass: road left edge → grass outer left
        grass_l_verts.append((lx[i],  float(ys[i]), lz[i]))
        grass_l_verts.append((glx[i], oy,           glz[i]))
        # Right grass: road right edge → grass outer right
        grass_r_verts.append((rx[i],  float(ys[i]), rz[i]))
        grass_r_verts.append((grx[i], oy,           grz[i]))
        grass_uvs.append((0.0, uc))
        grass_uvs.append((1.0, uc))

    # Terrain skirt strips: grass outer edge → 80m out, dropping to -2m.
    # Gives the surroundings (buildings/trees) ground to stand on and
    # something to land on if you leave the road.
    skirt_l_verts, skirt_r_verts, skirt_uvs = [], [], []
    s_drop = SKIRT_DROP
    for i in range(n_pts):
        uc = i / (n_pts-1) * total_length * ts * 0.25   # bigger texture tiles
        iy = float(ys[i]) - g_drop
        oy = float(ys[i]) - s_drop
        skirt_l_verts.append((glx[i],  iy, glz[i]))
        skirt_l_verts.append((slx_[i], oy, slz_[i]))
        skirt_r_verts.append((grx[i],  iy, grz[i]))
        skirt_r_verts.append((srx_[i], oy, srz_[i]))
        skirt_uvs.append((0.0, uc))
        skirt_uvs.append((1.0, uc))

    for i in range(n_pts - 1):
        a,b,c,d = i*2, i*2+1, i*2+2, i*2+3
        grass_l_faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
        grass_l_faces.append((a+1, c+1, d+1, a+1, c+1, d+1))
        grass_r_faces.append((a+1, d+1, b+1, a+1, d+1, b+1))
        grass_r_faces.append((a+1, c+1, d+1, a+1, c+1, d+1))

    # Stats
    dx2 = np.gradient(dxs); dz2 = np.gradient(dzs)
    curv = np.abs(dxs*dz2 - dzs*dx2) / (lengths_xz**3 + 1e-9)
    mc   = float(np.percentile(curv, 99))
    corners = int(np.sum(np.diff((curv > mc*0.15).astype(int)) > 0))
    dys = np.diff(ys)
    has_elev = bool(elev_profile and elev_profile.get("elevs")) or \
               (elevations is not None and any(e != 0 for e in (elevations or [])))

    return {
        "vertices": vertices,
        "uvs": uvs,
        "faces": faces,
        "grass_l_verts": grass_l_verts,
        "grass_r_verts": grass_r_verts,
        "grass_uvs": grass_uvs,
        "skirt_l_verts": skirt_l_verts,
        "skirt_r_verts": skirt_r_verts,
        "skirt_uvs": skirt_uvs,
        "grass_l_faces": grass_l_faces,
        "grass_r_faces": grass_r_faces,
        "centerline": list(zip(xs.tolist(), ys.tolist(), zs.tolist())),
        "elevation_profile": ys.tolist(),
        # projection parameters so surroundings can be placed in the
        # same local coordinate space
        "proj": {"mid_lat": float(mid_lat), "mid_lon": float(mid_lon),
                 "ox": ox, "oz": oz},
        "stats": {
            "length_m":       round(total_length, 1),
            "length_km":      round(total_length / 1000, 2),
            "point_count":    n_pts,
            "corners":        corners,
            "road_width":     road_width,
            "elev_min_m":     round(float(ys.min()), 1),
            "elev_max_m":     round(float(ys.max()), 1),
            "elev_range_m":   round(float(ys.max()-ys.min()), 1),
            "total_climb_m":  round(float(np.sum(dys[dys>0])), 1),
            "total_descent_m":round(float(np.sum(-dys[dys<0])), 1),
            "has_elevation":  has_elev,
        }
    }


# ─── OBJ Builder ──────────────────────────────────────────────────────────────

def build_obj(mesh: dict, track_name: str) -> tuple:
    """
    Build Wavefront OBJ + MTL.
    Road surface as one object (o 1ROAD).
    AC marker cubes as separate objects using correct A-to-B naming.

    For an open road (A-to-B stage), the required objects are:
      AC_AB_START_L / AC_AB_START_R  — start line left/right posts
      AC_AB_FINISH_L / AC_AB_FINISH_R — finish line left/right posts
      AC_PIT_0                        — pit/practice spawn
      AC_HOTLAP_START_0               — hotlap spawn
      AC_START_0                      — race spawn

    All cubes placed 1m above road surface, oriented with Z pointing
    in the direction of travel (AC reads heading from the cube's Z axis).
    """
    verts   = mesh["vertices"]
    uvs_raw = mesh["uvs"]
    faces   = mesh["faces"]
    cl      = mesh["centerline"]
    stats   = mesh["stats"]

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    hw     = stats["road_width"] / 2.0
    p0, p1 = cl[0], cl[min(1, len(cl)-1)]
    s_hdg  = hdg(p0, p1)
    hr     = math.radians(s_hdg)

    # Start line — at road start, 1m above surface, left and right of road
    sl_x = float(cl[0][0]) - math.sin(hr + math.pi/2) * hw
    sl_z = float(cl[0][2]) - math.cos(hr + math.pi/2) * hw
    sr_x = float(cl[0][0]) + math.sin(hr + math.pi/2) * hw
    sr_z = float(cl[0][2]) + math.cos(hr + math.pi/2) * hw
    start_y = float(cl[0][1]) + 1.0   # 1m above surface

    # Finish line — at road end
    pe0, pe1 = cl[-1], cl[min(len(cl)-2, len(cl)-1)]
    e_hdg = hdg(cl[-2], cl[-1])
    ehr   = math.radians(e_hdg)
    fl_x  = float(cl[-1][0]) - math.sin(ehr + math.pi/2) * hw
    fl_z  = float(cl[-1][2]) - math.cos(ehr + math.pi/2) * hw
    fr_x  = float(cl[-1][0]) + math.sin(ehr + math.pi/2) * hw
    fr_z  = float(cl[-1][2]) + math.cos(ehr + math.pi/2) * hw
    end_y = float(cl[-1][1]) + 1.0

    # Pit and spawn — 10m along road, 1m above, on road surface
    si    = min(10, len(cl)-1)
    sx    = float(cl[si][0])
    sz    = float(cl[si][2])
    sy    = float(cl[si][1]) + 1.0

    # Pit — left side of road near start
    px = float(cl[0][0]) - math.sin(hr + math.pi/2) * (hw * 0.5)
    pz = float(cl[0][2]) - math.cos(hr + math.pi/2) * (hw * 0.5)
    py = float(cl[0][1]) + 1.0

    markers = [
        ("AC_AB_START_L",    sl_x, start_y, sl_z, s_hdg),
        ("AC_AB_START_R",    sr_x, start_y, sr_z, s_hdg),
        ("AC_AB_FINISH_L",   fl_x, end_y,   fl_z, e_hdg),
        ("AC_AB_FINISH_R",   fr_x, end_y,   fr_z, e_hdg),
        ("AC_START_0",       sx,   sy,       sz,   s_hdg),
        ("AC_HOTLAP_START_0",sx,   sy,       sz,   s_hdg),
        ("AC_PIT_0",         px,   py,       pz,   s_hdg),
    ]

    lines = [
        f"# AC Road Tool - {track_name}",
        f"# A-to-B road track",
        f"mtllib {track_name}.mtl",
        "",
    ]

    # ── Road surface ──────────────────────────────────────────────────────
    lines.append(f"o 1ROAD")
    for v in verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl road")
    for f in faces:
        v1, v2, v3 = f[0], f[1], f[2]
        u1, u2, u3 = f[3], f[4], f[5]
        lines.append(f"f {v1}/{u1}/1 {v2}/{u2}/1 {v3}/{u3}/1")
    lines.append("")

    # ── Grass strips (left and right) ─────────────────────────────────────
    # 2m wide grass on each side of the road. Named 1GRASS so AC applies
    # the built-in GRASS surface physics (low grip, dirt).
    grass_l_verts = mesh["grass_l_verts"]
    grass_r_verts = mesh["grass_r_verts"]
    grass_uvs_raw = mesh["grass_uvs"]
    grass_l_faces = mesh["grass_l_faces"]
    grass_r_faces = mesh["grass_r_faces"]

    base_v = len(verts) + 1
    base_vt = len(uvs_raw) + 1

    # Left grass
    lines.append("o 1GRASS_L")
    for v in grass_l_verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in grass_uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl grass")
    for f in grass_l_faces:
        v1 = f[0] + base_v - 1
        v2 = f[1] + base_v - 1
        v3 = f[2] + base_v - 1
        u1 = f[3] + base_vt - 1
        u2 = f[4] + base_vt - 1
        u3 = f[5] + base_vt - 1
        lines.append(f"f {v1}/{u1}/2 {v2}/{u2}/2 {v3}/{u3}/2")
    lines.append("")

    base_v += len(grass_l_verts)
    base_vt += len(grass_uvs_raw)

    # Right grass
    lines.append("o 1GRASS_R")
    for v in grass_r_verts:
        lines.append(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
    for u in grass_uvs_raw:
        lines.append(f"vt {u[0]:.4f} {u[1]:.4f}")
    lines.append("vn 0.0000 1.0000 0.0000")
    lines.append("usemtl grass")
    for f in grass_r_faces:
        v1 = f[0] + base_v - 1
        v2 = f[1] + base_v - 1
        v3 = f[2] + base_v - 1
        u1 = f[3] + base_vt - 1
        u2 = f[4] + base_vt - 1
        u3 = f[5] + base_vt - 1
        lines.append(f"f {v1}/{u1}/3 {v2}/{u2}/3 {v3}/{u3}/3")
    lines.append("")

    # Update base_v for potential future objects
    base_v += len(grass_r_verts)

    # NOTE: AC spawn points are NOT in the OBJ file.
    # They are created as Blender empties by setup_in_blender.py.
    # OBJ cannot store object transforms, and AC reads spawn position
    # from the node transform — so empties must be created in Blender.

    obj_content = "\n".join(lines)

    mtl_content = (
        "# AC Road Tool\n"
        "newmtl road\n"
        "Ka 0.2 0.2 0.2\n"
        "Kd 0.5 0.5 0.5\n"
        "Ks 0.0 0.0 0.0\n"
        "d 1.0\n"
        "illum 1\n"
        "map_Kd road.png\n"
        "\n"
        "newmtl grass\n"
        "Ka 0.1 0.2 0.1\n"
        "Kd 0.2 0.5 0.2\n"
        "Ks 0.0 0.0 0.0\n"
        "d 1.0\n"
        "illum 1\n"
        "map_Kd grass.png\n"
    )

    return obj_content, mtl_content





# ─── Data Files ───────────────────────────────────────────────────────────────

def build_surfaces_ini() -> str:
    # Meshes are named 1ROAD / 1GRASS_L / 1GRASS_R, which bind to AC's
    # BUILT-IN default surfaces (KEY=ROAD, KEY=GRASS) defined in
    # assettocorsa/system/data/surfaces.ini — correct grip, sounds and
    # FMOD events out of the box.
    #
    # Redefining ROAD/GRASS here previously caused FMOD errors
    # (event:/surfaces/... not found) because of bad WAV= values.
    # An empty track surfaces.ini means "use the system defaults".
    return (
        "; AC Road Tool — intentionally empty.\n"
        "; Mesh prefixes 1ROAD / 1GRASS use AC's built-in default surfaces\n"
        "; from system/data/surfaces.ini (correct grip + sounds).\n"
        "; Add [SURFACE_N] sections here only for CUSTOM surface keys.\n"
    )


def build_track_json(track_name: str, stats: dict) -> str:
    import json as _j
    name = track_name.replace('_', ' ').title()
    elev = f" Elevation {stats.get('elev_range_m',0):.0f}m range." if stats.get('has_elevation') else ''
    return _j.dumps({
        "name": name,
        "description": f"Generated from OpenStreetMap. {stats.get('length_km',0)} km.{elev}",
        "tags": ["generated", "osm", "road"],
        "geotags": [], "country": "", "city": "",
        "length":   str(int(stats.get('length_m', 1000))),
        "width":    str(int(stats.get('road_width', 8))),
        "pitboxes": "1", "run": "",
        "author": "AC Road Tool", "version": "1.0", "url": ""
    }, indent=2)


# ─── PNG Encoding (stdlib only) ───────────────────────────────────────────────

def _png_encode(width: int, height: int, pixels: bytes, rgba: bool = False) -> bytes:
    """Encode raw pixel bytes (RGB or RGBA rows, no filter bytes) as PNG."""
    import zlib as _zlib
    def chunk(name, data):
        c = name + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', _zlib.crc32(c) & 0xffffffff)
    bpp = 4 if rgba else 3
    color_type = 6 if rgba else 2
    stride = width * bpp
    rows = b''.join(
        b'\x00' + pixels[y*stride:(y+1)*stride] for y in range(height)
    )
    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, color_type, 0, 0, 0))
            + chunk(b'IDAT', _zlib.compress(rows, 6))
            + chunk(b'IEND', b''))


# ─── Procedural Textures ──────────────────────────────────────────────────────

def build_road_texture(size: int = 256) -> bytes:
    """Asphalt with edge lines and a dashed centre line. V axis = along road."""
    rng = np.random.default_rng(42)
    noise = rng.integers(-9, 9, size=(size, size, 1))
    base = np.full((size, size, 3), (57, 57, 60), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)

    # White edge lines (~4% in from each edge)
    e0 = max(2, size // 26)
    ew = max(2, size // 64)
    img[:, e0:e0 + ew, :] = 225
    img[:, size - e0 - ew:size - e0, :] = 225

    # Dashed white centre line (painted on half the tile → dashes when tiled)
    c = size // 2
    half = size // 2
    cw = max(1, size // 128)
    img[0:half, c - cw - 1:c + cw + 1, :] = 210

    return _png_encode(size, size, img.tobytes())


def build_grass_texture(size: int = 128) -> bytes:
    rng = np.random.default_rng(7)
    noise = rng.integers(-14, 14, size=(size, size, 1))
    base = np.full((size, size, 3), (66, 105, 48), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    return _png_encode(size, size, img.tobytes())


def build_building_texture(size: int = 128) -> bytes:
    """Light render/plaster with faint horizontal floor lines + window grid."""
    rng = np.random.default_rng(11)
    noise = rng.integers(-7, 7, size=(size, size, 1))
    base = np.full((size, size, 3), (176, 170, 160), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    # window grid (dark rectangles)
    step = size // 4
    for wy in range(step // 3, size, step):
        for wx in range(step // 3, size, step):
            img[wy:wy + step // 3, wx:wx + step // 3, :] = (72, 82, 95)
    return _png_encode(size, size, img.tobytes())


def build_tree_texture(size: int = 64) -> bytes:
    rng = np.random.default_rng(23)
    noise = rng.integers(-18, 18, size=(size, size, 1))
    base = np.full((size, size, 3), (44, 84, 38), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    return _png_encode(size, size, img.tobytes())


def build_terrain_texture(size: int = 128) -> bytes:
    """Scrubbier, browner green than the mown verge grass."""
    rng = np.random.default_rng(31)
    noise = rng.integers(-20, 20, size=(size, size, 1))
    base = np.full((size, size, 3), (74, 92, 52), dtype=np.int16) + noise
    img = np.clip(base, 0, 255).astype(np.uint8)
    return _png_encode(size, size, img.tobytes())


# ─── Track Map (map.png + map.ini + ui images) ────────────────────────────────

def _render_route(mesh: dict, size: int, margin: int, line_px: float,
                  bg=(0, 0, 0, 0), fg=(255, 255, 255, 255)) -> tuple:
    """Render top-down route. Returns (rgba_bytes, min_x, min_z, scale)."""
    cl = mesh["centerline"]
    xs = np.array([p[0] for p in cl]); zs = np.array([p[2] for p in cl])
    min_x, max_x = float(xs.min()), float(xs.max())
    min_z, max_z = float(zs.min()), float(zs.max())
    ext = max(max_x - min_x, max_z - min_z, 1.0)
    scale = ext / (size - 2 * margin)          # metres per pixel

    canvas = np.zeros((size, size, 4), dtype=np.uint8)
    canvas[:, :] = bg

    px = ((xs - min_x) / scale + margin).astype(int)
    py = ((zs - min_z) / scale + margin).astype(int)

    r = max(1, int(round(line_px / 2)))
    yy, xx = np.mgrid[-r:r+1, -r:r+1]
    disk = (xx**2 + yy**2) <= r*r
    dy, dx = np.nonzero(disk)
    dy = dy - r; dx = dx - r

    for cx, cy in zip(px, py):
        ys_ = np.clip(cy + dy, 0, size-1)
        xs_ = np.clip(cx + dx, 0, size-1)
        canvas[ys_, xs_] = fg

    return canvas.tobytes(), min_x, min_z, scale


def build_track_map(mesh: dict) -> tuple:
    """Returns (map_png, map_ini, outline_png, preview_png)."""
    size, margin = 1024, 32
    road_w = mesh["stats"]["road_width"]

    # First pass to get scale, second pass with road-width-proportional line
    _, min_x, min_z, scale = _render_route(mesh, size, margin, 1)
    line_px = max(4.0, road_w / scale)
    rgba, min_x, min_z, scale = _render_route(mesh, size, margin, line_px)
    map_png = _png_encode(size, size, rgba, rgba=True)

    map_ini = (
        "[PARAMETERS]\n"
        f"WIDTH={size}\n"
        f"HEIGHT={size}\n"
        f"X_OFFSET={margin * scale - min_x:.3f}\n"
        f"Z_OFFSET={margin * scale - min_z:.3f}\n"
        "MARGIN=20\n"
        f"SCALE_FACTOR={scale:.5f}\n"
        "MAX_SIZE=1600\n"
        "MIN_SIZE=300\n"
        "DRAWING_SIZE=10\n"
    )

    # ui/outline.png — route on transparent, smaller
    o_rgba, *_ = _render_route(mesh, 512, 24, 6)
    outline_png = _png_encode(512, 512, o_rgba, rgba=True)

    # ui/preview.png — route on dark background
    p_rgba, *_ = _render_route(mesh, 565, 40, 8,
                               bg=(24, 26, 30, 255), fg=(240, 240, 240, 255))
    preview_png = _png_encode(565, 565, p_rgba, rgba=True)

    return map_png, map_ini, outline_png, preview_png


# ─── KN5 Writer (pure Python — no Blender, no ksEditor) ───────────────────────
# Binary format of Assetto Corsa's KN5 model files, as implemented by the
# open-source GPL Blender exporters (Thomas Hagnhofer / moppius fork).
# Reimplemented here for direct export from this tool.

class _KN5:
    def __init__(self):
        self.buf = io.BytesIO()

    def u32(self, v):  self.buf.write(struct.pack('<I', int(v)))
    def i32(self, v):  self.buf.write(struct.pack('<i', int(v)))
    def u16(self, v):  self.buf.write(struct.pack('<H', int(v)))
    def f32(self, v):  self.buf.write(struct.pack('<f', float(v)))
    def byte(self, v): self.buf.write(struct.pack('<B', int(v)))
    def flag(self, v): self.buf.write(struct.pack('<?', bool(v)))
    def s(self, text):
        b = text.encode('utf-8')
        self.u32(len(b)); self.buf.write(b)
    def blob(self, b):
        self.u32(len(b)); self.buf.write(b)
    def v2(self, v): self.buf.write(struct.pack('<2f', *[float(x) for x in v]))
    def v3(self, v): self.buf.write(struct.pack('<3f', *[float(x) for x in v]))
    def v4(self, v): self.buf.write(struct.pack('<4f', *[float(x) for x in v]))

    # D3D row-major: rows are basis vectors [right, up, forward, position]
    def matrix(self, right, up, fwd, pos):
        for vec, w in ((right, 0.0), (up, 0.0), (fwd, 0.0), (pos, 1.0)):
            self.f32(vec[0]); self.f32(vec[1]); self.f32(vec[2]); self.f32(w)

    def material(self, name, shader, texture_name,
                 ambient=0.5, diffuse=0.45, specular=0.05, spec_exp=20.0):
        self.s(name)
        self.s(shader)
        self.byte(0)      # alphaBlendMode: Opaque
        self.flag(False)  # alphaTested
        self.i32(0)       # depthMode: DepthNormal
        props = [("ksAmbient", ambient), ("ksDiffuse", diffuse),
                 ("ksSpecular", specular), ("ksSpecularEXP", spec_exp)]
        self.u32(len(props))
        for pname, a in props:
            self.s(pname)
            self.f32(a)
            self.v2((0, 0)); self.v3((0, 0, 0)); self.v4((0, 0, 0, 0))
        self.u32(1)                    # one texture mapping
        self.s("txDiffuse")
        self.u32(0)                    # slot
        self.s(texture_name)

    def dummy_node(self, name, child_count, pos=(0, 0, 0), heading_deg=0.0):
        h = math.radians(heading_deg)
        right = ( math.cos(h), 0.0, -math.sin(h))
        up    = ( 0.0,         1.0,  0.0)
        fwd   = ( math.sin(h), 0.0,  math.cos(h))
        self.u32(1)               # class: Node
        self.s(name)
        self.u32(child_count)
        self.flag(True)           # active
        self.matrix(right, up, fwd, pos)

    def mesh_node(self, name, verts, indices, material_id):
        """verts: list of (pos3, normal3, uv2, tangent3). indices: flat tri list."""
        if len(verts) > 65535:
            raise ValueError(f"{name}: {len(verts)} verts exceeds 65535 limit")
        self.u32(2)               # class: Mesh
        self.s(name)
        self.u32(0)               # children (none allowed for meshes)
        self.flag(True)           # active
        self.flag(True)           # castShadows
        self.flag(True)           # visible
        self.flag(False)          # transparent
        self.u32(len(verts))
        for pos, nrm, uv, tan in verts:
            self.v3(pos); self.v3(nrm); self.v2(uv); self.v3(tan)
        self.u32(len(indices))
        for i in indices:
            self.u16(i)
        self.u32(material_id)
        self.u32(0)               # layer
        self.f32(0.0)             # lodIn
        self.f32(0.0)             # lodOut (0 = always visible)
        # bounding sphere from bbox (matches reference exporter behaviour)
        px = [v[0][0] for v in verts]; py = [v[0][1] for v in verts]; pz = [v[0][2] for v in verts]
        cx = (min(px)+max(px))/2; cy = (min(py)+max(py))/2; cz = (min(pz)+max(pz))/2
        radius = max(max(px)-min(px), max(py)-min(py), max(pz)-min(pz))
        self.v3((cx, cy, cz))
        self.f32(radius)
        self.flag(True)           # renderable


def _strip_to_kn5_verts(pairs_flat, uvs, centerline):
    """
    Convert an alternating (A-side, B-side) vertex strip into KN5 vertex
    tuples with smooth up-facing normals and along-road tangents.
    Returns (verts, flip_winding).
    """
    n = len(pairs_flat) // 2
    verts = []
    for i in range(n):
        a = pairs_flat[2*i]
        b = pairs_flat[2*i + 1]
        i0, i1 = max(0, i-1), min(n-1, i+1)
        c0, c1 = centerline[i0], centerline[i1]
        fwd = (c1[0]-c0[0], c1[1]-c0[1], c1[2]-c0[2])
        fl = math.sqrt(fwd[0]**2 + fwd[1]**2 + fwd[2]**2) or 1.0
        fwd = (fwd[0]/fl, fwd[1]/fl, fwd[2]/fl)
        lat = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        # normal = fwd × lat, forced upward
        nx = fwd[1]*lat[2] - fwd[2]*lat[1]
        ny = fwd[2]*lat[0] - fwd[0]*lat[2]
        nz = fwd[0]*lat[1] - fwd[1]*lat[0]
        nl = math.sqrt(nx*nx + ny*ny + nz*nz) or 1.0
        nx, ny, nz = nx/nl, ny/nl, nz/nl
        if ny < 0:
            nx, ny, nz = -nx, -ny, -nz
        for v, u in ((a, uvs[2*i]), (b, uvs[2*i+1])):
            verts.append(((v[0], v[1], v[2]), (nx, ny, nz), u, fwd))

    # Winding check on first quad: geometric normal of tri (0,3,1) must be up
    if n >= 2:
        p0 = pairs_flat[0]; p3 = pairs_flat[3]; p1 = pairs_flat[1]
        e1 = (p3[0]-p0[0], p3[1]-p0[1], p3[2]-p0[2])
        e2 = (p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2])
        gy = e1[2]*e2[0] - e1[0]*e2[2]     # Y of e1×e2
        flip = gy < 0
    else:
        flip = False
    return verts, flip


def _strip_indices(n_pairs, flip):
    idx = []
    for i in range(n_pairs - 1):
        a, b, c, d = 2*i, 2*i+1, 2*i+2, 2*i+3
        if flip:
            idx.extend((a, b, d)); idx.extend((a, d, c))
        else:
            idx.extend((a, d, b)); idx.extend((a, c, d))
    return idx


def _chunk_strip(verts, uvs, centerline, base_name, max_pairs=30000):
    """Split long strips into ≤65k-vertex meshes: yields (name, verts, uvs, cl)."""
    n = len(verts) // 2
    if n <= max_pairs:
        yield base_name, verts, uvs, centerline
        return
    start = 0
    part = 0
    while start < n - 1:
        end = min(start + max_pairs, n)
        name = base_name if part == 0 else f"{base_name}_{part}"
        yield (name,
               verts[2*start:2*end],
               uvs[2*start:2*end],
               centerline[start:end])
        start = end - 1   # overlap one pair to avoid gaps
        part += 1


def build_kn5(mesh: dict, track_name: str, env_meshes: list = None,
              terrain_meshes: list = None) -> bytes:
    """Build a complete, ready-to-drive KN5 — no Blender, no ksEditor.
    env_meshes: [(name, kn5_verts, indices, material_key)] visuals.
    terrain_meshes: real-elevation lateral terrain (physical); when given,
    the synthetic flat skirt is skipped."""
    k = _KN5()
    k.buf.write(b"sc6969")
    k.u32(5)                                   # file version

    # ── Textures ──
    textures = [("road.png",     build_road_texture()),
                ("grass.png",    build_grass_texture()),
                ("building.png", build_building_texture()),
                ("tree.png",     build_tree_texture()),
                ("terrain.png",  build_terrain_texture())]
    k.i32(len(textures))
    for tex_name, data in textures:
        k.i32(1)                               # active
        k.s(tex_name)
        k.blob(data)

    # ── Materials ──
    mat_ids = {"road": 0, "grass": 1, "building": 2, "tree": 3, "terrain": 4}
    k.i32(5)
    k.material("road",     "ksPerPixel", "road.png",     specular=0.08, spec_exp=30.0)
    k.material("grass",    "ksPerPixel", "grass.png",    specular=0.01, spec_exp=5.0)
    k.material("building", "ksPerPixel", "building.png", specular=0.03, spec_exp=10.0)
    k.material("tree",     "ksPerPixel", "tree.png",     specular=0.01, spec_exp=5.0)
    k.material("terrain",  "ksPerPixel", "terrain.png",  specular=0.01, spec_exp=5.0)

    # ── Geometry ──
    cl = mesh["centerline"]
    meshes = []   # (name, verts, indices, material_id)

    def add_strip(base_name, vs_key_or_list, uvs_list, mat_id):
        vs_all = mesh[vs_key_or_list] if isinstance(vs_key_or_list, str) else vs_key_or_list
        out = []
        for name, vs, us, sub_cl in _chunk_strip(vs_all, uvs_list, cl, base_name):
            kv, flip = _strip_to_kn5_verts(vs, us, sub_cl)
            out.append((name, kv, _strip_indices(len(kv)//2, flip), mat_id))
        return out

    road_meshes = add_strip("1ROAD", "vertices", mesh["uvs"], 0)
    meshes.extend(road_meshes)
    grass_meshes = []
    for base_name, key in (("1GRASS_L", "grass_l_verts"), ("1GRASS_R", "grass_r_verts")):
        grass_meshes.extend(add_strip(base_name, key, mesh["grass_uvs"], 1))
    meshes.extend(grass_meshes)
    # Terrain: real-elevation grid when available, flat skirt as fallback —
    # physical (GRASS surface) either way, so leaving the road doesn't drop
    # the car into the void
    if terrain_meshes:
        for name, kv, idx, mat_key in terrain_meshes:
            meshes.append((name, kv, idx, mat_ids[mat_key]))
    elif mesh.get("skirt_l_verts"):
        for base_name, key in (("1GRASS_SL", "skirt_l_verts"),
                               ("1GRASS_SR", "skirt_r_verts")):
            meshes.extend(add_strip(base_name, key, mesh["skirt_uvs"],
                                    mat_ids["terrain"]))

    # Winding insurance: AC's front-face convention can't be verified
    # offline, so add a visual-only copy of each strip 3cm lower with
    # REVERSED winding + flipped normals. Whichever convention AC uses,
    # one of the two copies is visible from above. Names carry no digit
    # prefix and no surface key, so they have no physics.
    underlay_id = 0
    for name, kv, idx, mat_id in list(meshes):
        u_kv = [((p[0], p[1] - 0.03, p[2]), (-n[0], -n[1], -n[2]), uv, t)
                for p, n, uv, t in kv]
        u_idx = []
        for t0 in range(0, len(idx), 3):
            u_idx.extend((idx[t0], idx[t0+2], idx[t0+1]))   # reverse winding
        meshes.append((f"UNDERLAY_{underlay_id}", u_kv, u_idx, mat_id))
        underlay_id += 1

    # ── Environment (buildings + trees) — visual only, no physics ──
    for name, kv, idx, mat_key in (env_meshes or []):
        meshes.append((name, kv, idx, mat_ids[mat_key]))

    # ── Spawn nodes — position encoded in the NODE TRANSFORM, which is
    #    exactly what AC reads. This is the definitive fix for
    #    "NO POSITION DATA FOUND".
    stats = mesh["stats"]
    hw = stats["road_width"] / 2.0

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    s_hdg = hdg(cl[0], cl[min(1, len(cl)-1)])
    e_hdg = hdg(cl[-2], cl[-1])
    hr, ehr = math.radians(s_hdg), math.radians(e_hdg)

    # Ordering matters: the car must spawn BEHIND the start gate so the
    # timer starts when it crosses the line (not instantly on load).
    # spawn/pit ~5m in, start gate ~35m in, finish gate ~5m before the end.
    n_cl = len(cl)
    spawn_i = min(5,  n_cl - 1)
    pit_i   = min(8,  n_cl - 1)
    gate_i  = min(35, max(spawn_i + 5, int(n_cl * 0.05)))
    gate_i  = min(gate_i, n_cl - 2)
    fin_i   = max(n_cl - 6, gate_i + 1)

    g_hdg  = hdg(cl[gate_i], cl[min(gate_i + 1, n_cl - 1)])
    ghr    = math.radians(g_hdg)
    f_hdg  = hdg(cl[max(fin_i - 1, 0)], cl[fin_i])
    fhr    = math.radians(f_hdg)

    spawn = (float(cl[spawn_i][0]), float(cl[spawn_i][1]) + 1.5, float(cl[spawn_i][2]))
    pit   = (float(cl[pit_i][0]),   float(cl[pit_i][1])   + 1.5, float(cl[pit_i][2]))

    def edge(p, h, side, dist):
        return (p[0] + side*math.sin(h + math.pi/2)*dist,
                p[1] + 1.5,
                p[2] + side*math.cos(h + math.pi/2)*dist)

    dummies = [
        ("AC_AB_START_L",     edge(cl[gate_i], ghr, +1, hw), g_hdg),
        ("AC_AB_START_R",     edge(cl[gate_i], ghr, -1, hw), g_hdg),
        ("AC_AB_FINISH_L",    edge(cl[fin_i],  fhr, +1, hw), f_hdg),
        ("AC_AB_FINISH_R",    edge(cl[fin_i],  fhr, -1, hw), f_hdg),
        ("AC_START_0",        spawn, s_hdg),
        ("AC_HOTLAP_START_0", spawn, s_hdg),
        ("AC_PIT_0",          pit,   s_hdg),
    ]

    # ── Node tree ── root(identity) → meshes + spawn dummies
    k.dummy_node(track_name, len(meshes) + len(dummies))
    for name, kv, idx, mat_id in meshes:
        k.mesh_node(name, kv, idx, mat_id)
    for name, pos, heading in dummies:
        k.dummy_node(name, 0, pos, heading)

    return k.buf.getvalue()



# ─── Export ───────────────────────────────────────────────────────────────────


def build_blender_script(mesh: dict, track_name: str) -> str:
    """
    Generate a Blender Python script that creates AC spawn point empties
    with the correct configuration for Assetto Corsa.

    Run this AFTER importing the OBJ. It creates empties (not mesh cubes)
    at suggested positions along the road. The user can then move them
    to fine-tune placement before exporting FBX.

    AC spawn point rules (from community research):
      - Empty or cube, named AC_START_0, AC_PIT_0, etc.
      - Axis: local Y up, local Z forward (direction of travel)
      - In Blender this means: rotation_euler X=90°, Y=0°, Z=heading
      - Scale: 0.01 (DO NOT apply scale)
      - DO NOT apply rotation (Ctrl+A) — AC reads the unapplied transform
      - Height: 1-2m above road surface
    """
    cl = mesh["centerline"]
    stats = mesh["stats"]
    hw = stats["road_width"] / 2.0

    def hdg(p1, p2):
        return math.degrees(math.atan2(p2[0]-p1[0], p2[2]-p1[2]))

    s_hdg = hdg(cl[0], cl[min(1, len(cl)-1)])
    hr = math.radians(s_hdg)
    e_hdg = hdg(cl[-2], cl[-1])
    ehr = math.radians(e_hdg)

    # Spawn positions — 10 points along road, 1.5m above surface
    si = min(10, len(cl)-1)
    sx, sy, sz = float(cl[si][0]), float(cl[si][1]) + 1.5, float(cl[si][2])

    # Gate posts — at road edges
    sl_x = float(cl[0][0]) - math.sin(hr + math.pi/2) * hw
    sl_z = float(cl[0][2]) - math.cos(hr + math.pi/2) * hw
    sr_x = float(cl[0][0]) + math.sin(hr + math.pi/2) * hw
    sr_z = float(cl[0][2]) + math.cos(hr + math.pi/2) * hw
    start_y = float(cl[0][1]) + 1.5

    fl_x = float(cl[-1][0]) - math.sin(ehr + math.pi/2) * hw
    fl_z = float(cl[-1][2]) - math.cos(ehr + math.pi/2) * hw
    fr_x = float(cl[-1][0]) + math.sin(ehr + math.pi/2) * hw
    fr_z = float(cl[-1][2]) + math.cos(ehr + math.pi/2) * hw
    end_y = float(cl[-1][1]) + 1.5

    # Pit — left side of road near start
    px = float(cl[0][0]) - math.sin(hr + math.pi/2) * (hw * 0.5)
    pz = float(cl[0][2]) - math.cos(hr + math.pi/2) * (hw * 0.5)
    py = float(cl[0][1]) + 1.5

    markers = [
        ("AC_AB_START_L",    sl_x, start_y, sl_z, s_hdg),
        ("AC_AB_START_R",    sr_x, start_y, sr_z, s_hdg),
        ("AC_AB_FINISH_L",   fl_x, end_y,   fl_z, e_hdg),
        ("AC_AB_FINISH_R",   fr_x, end_y,   fr_z, e_hdg),
        ("AC_START_0",       sx,   sy,       sz,   s_hdg),
        ("AC_HOTLAP_START_0",sx,   sy,       sz,   s_hdg),
        ("AC_PIT_0",         px,   py,       pz,   s_hdg),
    ]

    lines = [
        "import bpy, math",
        "",
        f"# AC Road Tool — spawn point setup for: {track_name}",
        "#",
        "# This script creates AC spawn point empties with correct settings.",
        "# Run AFTER importing the OBJ file.",
        "#",
        "# The empties are placed at suggested positions — you can move them",
        "# to wherever you want on the track. Just keep these rules:",
        "#   - DO NOT apply rotation (no Ctrl+A > Rotation)",
        "#   - DO NOT apply scale (no Ctrl+A > Scale)",
        "#   - Keep them 1-2m above the road surface",
        "#   - The arrow points in the direction the car will face",
        "",
        "spawn_points = [",
    ]

    for name, x, y, z, h in markers:
        lines.append(f'    ("{name}", {x:.4f}, {y:.4f}, {z:.4f}, {h:.2f}),')

    lines.extend([
        "]",
        "",
        "created = []",
        "for name, x, y, z, heading in spawn_points:",
        "    empty = bpy.data.objects.new(name, None)",
        "    bpy.context.collection.objects.link(empty)",
        "    empty.empty_display_type = 'SINGLE_ARROW'",
        "    empty.empty_display_size = 2.0",
        "    empty.location = (x, y, z)",
        "    # X=90° makes local Y point up, Z becomes forward",
        "    # Z=heading rotates to face along the road",
        "    empty.rotation_euler = (math.radians(90), 0, math.radians(heading))",
        "    empty.scale = (0.01, 0.01, 0.01)",
        "    created.append(name)",
        '    print(f"  Created {name} at ({x:.1f}, {y:.1f}, {z:.1f}) heading={heading:.1f}")',
        "",
        "# Show axis on empties so you can verify orientation",
        "for obj in bpy.data.objects:",
        "    if obj.name in created:",
        "        obj.show_axis = True",
        "",
        f'print("\\nCreated {len(markers)} spawn points for {track_name}")',
        'print("You can move them — just DO NOT apply rotation or scale.")',
        'print("The arrow shows the direction the car will face.")',
    ])

    return "\n".join(lines)



def build_track_files(mesh: dict, track_name: str, env_meshes: list = None,
                      terrain_meshes: list = None) -> dict:
    """Build every file of a complete, ready-to-drive AC track.
    Returns {relative_path: bytes}."""
    stats = mesh["stats"]
    files = {}

    # ── The KN5 itself — direct export, ready to drive ──
    files[f"{track_name}.kn5"] = build_kn5(mesh, track_name, env_meshes,
                                           terrain_meshes)

    files["models.ini"] = (
        f"[MODEL_0]\nFILE={track_name}.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n"
    ).encode()

    # ── data/ ──
    files["data/surfaces.ini"] = build_surfaces_ini().encode()

    map_png, map_ini, outline_png, preview_png = build_track_map(mesh)
    files["data/map.ini"] = map_ini.encode()
    files["map.png"]      = map_png

    # ── ui/ ──
    files["ui/ui_track.json"] = build_track_json(track_name, stats).encode()
    files["ui/preview.png"]   = preview_png
    files["ui/outline.png"]   = outline_png

    # ai/ folder must exist; AC records fast_lane here on first hotlap
    files["ai/.placeholder"] = b""

    # ── extras/ — optional Blender workflow for customisation ──
    obj, mtl = build_obj(mesh, track_name)
    if len(obj) <= 5 * 1024 * 1024:   # skip giant OBJs on long tracks
        files[f"extras/{track_name}.obj"] = obj.encode()
        files[f"extras/{track_name}.mtl"] = mtl.encode()
    files["extras/setup_in_blender.py"] = build_blender_script(mesh, track_name).encode()
    files["extras/README_EXTRAS.txt"] = (
        "These files are OPTIONAL - the track already works out of the box.\n"
        "Use them only if you want to customise the mesh in Blender and\n"
        "rebuild the KN5 yourself via FBX + ksEditor.\n"
    ).encode()

    files["README.txt"] = f"""AC Road Tool Export
===================
Track: {track_name}  |  Length: {stats['length_km']} km  |  Width: {stats['road_width']} m

THIS TRACK IS READY TO DRIVE - no Blender or ksEditor needed.
The .kn5 was generated directly with road, grass, textures and
all spawn points built in.

INSTALL
  1. Delete any previous version of this track folder.
  2. Extract the zip into your Assetto Corsa root folder:
     C:\\...\\steamapps\\common\\assettocorsa\\
     (so this folder ends up in assettocorsa/content/tracks/)
  3. Launch in HOTLAP or PRACTICE mode.
     (Race needs an AI line: drive the road once in Hotlap mode,
      AC saves a fast_lane candidate into the ai/ folder.)

If the car spawns facing the wrong way, tell the tool author -
the heading convention flips in one place.

extras/ contains an optional Blender workflow for customisation.
""".encode()

    return files


def export_ac_package(coords: list, road_width: float,
                      smooth_factor: float, track_name: str,
                      install_path: str = None,
                      include_env: bool = True) -> dict:
    print(f"  [export] Fetching elevation profile…")
    elev_profile = fetch_elevation_profile(coords)
    print(f"  [export] Elevation {'OK' if elev_profile else 'flat'}")

    mesh = process_road(coords, road_width, smooth_factor,
                        elev_profile=elev_profile)
    if "error" in mesh:
        return mesh

    # ── Real terrain beside the road (cliffs, valleys, hillsides) ──
    terrain_meshes = None
    ground_pts = None
    if elev_profile:
        try:
            tgrid = fetch_terrain_grid(mesh)
            if tgrid:
                terrain_meshes = build_terrain_meshes(mesh, tgrid)
                ground_pts = [v[0] for _, kv, _, _ in terrain_meshes for v in kv]
                ground_pts += list(mesh["centerline"])
                print(f"  [export] Real terrain: "
                      f"{sum(len(kv) for _, kv, _, _ in terrain_meshes)} verts")
        except Exception as e:
            print(f"  [export] Terrain grid skipped: {e}")

    env_meshes = None
    n_bldg = n_tree = 0
    if include_env:
        try:
            print(f"  [export] Fetching surroundings from OSM…")
            surroundings = fetch_surroundings(coords)
            env = build_environment_meshes(surroundings, mesh, ground_pts)
            env_meshes = env["meshes"]
            n_bldg = env["n_buildings"]
            n_tree = env["n_trees"]
            print(f"  [export] Environment: {n_bldg} buildings, {n_tree} trees "
                  f"(incl. forest scatter)")
        except Exception as e:
            print(f"  [export] Surroundings skipped: {e}")

    stats = mesh["stats"]
    files = build_track_files(mesh, track_name, env_meshes, terrain_meshes)

    base = f"content/tracks/{track_name}/"
    zip_buf = io.BytesIO()
    with zipfile.ZipFile(zip_buf, 'w', zipfile.ZIP_DEFLATED) as zf:
        for rel, data in files.items():
            zf.writestr(base + rel, data)

    zip_bytes = zip_buf.getvalue()
    with open(os.path.join(OUTPUT_DIR, f"{track_name}.zip"), 'wb') as f:
        f.write(zip_bytes)

    # ── Direct install (true 1-click) ──
    installed_to = None
    install_error = None
    if install_path:
        try:
            root = os.path.expanduser(install_path.strip())
            # Accept either the AC root or the tracks folder directly
            if os.path.basename(root.rstrip("/\\")).lower() == "tracks":
                tracks_dir = root
            elif os.path.isdir(os.path.join(root, "content", "tracks")):
                tracks_dir = os.path.join(root, "content", "tracks")
            elif os.path.isdir(root):
                tracks_dir = os.path.join(root, "content", "tracks")
                os.makedirs(tracks_dir, exist_ok=True)
            else:
                raise FileNotFoundError(f"Folder not found: {root}")

            track_dir = os.path.join(tracks_dir, track_name)
            if os.path.isdir(track_dir):
                shutil.rmtree(track_dir)   # clean old version incl. cached ai
            for rel, data in files.items():
                dest = os.path.join(track_dir, rel.replace("/", os.sep))
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                with open(dest, 'wb') as f:
                    f.write(data)
            installed_to = track_dir
            print(f"  [export] Installed directly to {track_dir}")
        except Exception as e:
            install_error = str(e)
            print(f"  [export] Direct install failed: {e}")

    profile = mesh["elevation_profile"]
    return {
        "stats":             stats,
        "filename":          f"{track_name}.zip",
        "size_kb":           round(len(zip_bytes) / 1024, 1),
        "kn5_kb":            round(len(files[f"{track_name}.kn5"]) / 1024, 1),
        "buildings":         n_bldg,
        "trees":             n_tree,
        "installed_to":      installed_to,
        "install_error":     install_error,
        "elevation_profile": profile[::max(1, len(profile)//500)],
    }


# ─── HTTP Handler ─────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path):
        with open(path, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "application/zip")
        self.send_header("Content-Length", len(data))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Disposition",
                         f'attachment; filename="{os.path.basename(path)}"')
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)

        if parsed.path == "/":
            with open(os.path.join(os.path.dirname(__file__),
                                   "static", "index.html"), 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif parsed.path == "/api/search":
            q = qs.get("q", [""])[0]
            self.send_json(search_osm_roads(q) if q else {"error": "No query"})

        elif parsed.path == "/api/geometry":
            t = qs.get("type", [""])[0]
            i = qs.get("id",   [""])[0]
            self.send_json(fetch_road_geometry(t, i) if t and i
                           else {"error": "Missing type/id"})

        elif parsed.path.startswith("/download/"):
            fname = parsed.path.replace("/download/", "")
            fpath = os.path.join(OUTPUT_DIR, fname)
            self.send_file(fpath) if os.path.exists(fpath) else self.send_json({"error": "Not found"}, 404)

        else:
            self.send_json({"error": "Not found"}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        body   = json.loads(self.rfile.read(length)) if length else {}

        if parsed.path == "/api/export":
            coords = body.get("coords", [])
            if not coords:
                self.send_json({"error": "No coords"}, 400)
                return
            rw    = float(body.get("road_width", 8.0))
            sm    = float(body.get("smooth", 0.3))
            name  = "".join(c for c in body.get("track_name", "my_road")
                            .replace(" ", "_").lower()
                            if c.isalnum() or c == "_")[:32]
            install_path = body.get("install_path") or None
            include_env  = bool(body.get("include_env", True))
            self.send_json(export_ac_package(coords, rw, sm, name,
                                             install_path, include_env))

        elif parsed.path == "/api/preview":
            coords = body.get("coords", [])
            if not coords:
                self.send_json({"error": "No coords"}, 400)
                return
            prof = fetch_elevation_profile(coords)
            mesh = process_road(coords,
                                float(body.get("road_width", 8.0)),
                                float(body.get("smooth", 0.3)),
                                elev_profile=prof)
            if "error" in mesh:
                self.send_json(mesh, 400)
                return
            p    = mesh["elevation_profile"]
            step = max(1, len(p) // 500)
            self.send_json({"stats": mesh["stats"], "elevation_profile": p[::step]})

        else:
            self.send_json({"error": "Not found"}, 404)


if __name__ == "__main__":
    print(f"\n  AC Road Tool")
    print(f"  ─────────────────────────────")
    print(f"  Open in browser: http://localhost:{PORT}")
    print(f"  Press Ctrl+C to stop\n")
    server = HTTPServer(("", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Server stopped.")
