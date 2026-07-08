# AC Road Tool

Turns a real-world road into a driveable Assetto Corsa track. You pick a
road on a map, click export, and it writes a finished `.kn5` track that
AC can load. There is no Blender or ksEditor step.

## Requirements

- Python 3.8 or newer
- Packages: `numpy`, `scipy`, `pyproj`
  (the launcher installs these automatically if they're missing)
- An internet connection (the tool pulls map, elevation, and terrain
  data from public web services)

## Running it

```
python launch.py
```

This starts a local server and opens the interface at
`http://localhost:8743` in your browser. Leave the terminal window open
while you use the tool; closing it stops the server. Press Ctrl+C in the
terminal to stop.

If the browser doesn't open on its own, go to `http://localhost:8743`
manually.

## Using it

1. Search for a road by name (for example "Gorge Road, South Australia").
2. Click a search result. The road's shape loads on the map.
3. The whole road loads, not just one segment. If only part of the road
   is relevant, drag the green START and red END markers along the road
   to trim it. The greyed-out dashed line shows the part being left off.
4. Set the road width and smoothing if the defaults don't suit.
5. Tick or untick "Include surroundings" (buildings and trees).
6. Optional: paste the path to your Assetto Corsa folder. If you do, the
   track installs straight into the game on export.
7. Click Export.

If you set the AC folder path, the track is written directly into
`assettocorsa/content/tracks/` and you can launch the game. If you
didn't, you get a zip instead — drag it into content manager.

Drive the track in Hotlap or Practice mode. Race mode needs an AI
racing line, which the track doesn't include. To get one, drive the road
once in Hotlap; AC records a lap line into the track's `ai/` folder that
Race mode can then use.

## How it works

The road shape comes from OpenStreetMap. A single road is usually stored
as many separate pieces there, so the tool looks up every piece with the
same name in the surrounding area and joins them into one continuous
route.

Elevation comes from public terrain data services (opentopodata.org and
open-meteo.com). Heights are sampled at even spacing along the road and
smoothed, so the surface follows the real hills without picking up steps
from the source data's coarse resolution.

From that centreline the tool builds the mesh: the road surface at your
chosen width, grass verges either side, and a wider terrain skirt beyond
that. When elevation data is available it also samples heights out to the
sides of the road and builds real terrain there, so nearby hillsides,
valleys, and cliff faces show up.

If surroundings are enabled, it pulls buildings and vegetation from
OpenStreetMap. Buildings are extruded from their real footprints using
their tagged height or floor count. Trees come from three sources:
individual mapped trees, tree rows along roadsides, and forest or park
areas. Because parks and forests are stored as areas rather than
individual trees, the tool scatters trees inside those areas, thinning
the scatter for open parkland versus dense woodland.

The output is written straight to AC's `.kn5` model format in Python.
The binary layout follows the open-source GPL Blender exporters by
Thomas Hagnhofer and moppius. Spawn points (start, pit, hotlap) are
written as nodes whose position lives in the node transform, which is
where AC reads spawn locations from. This is the part that a
Blender-then-ksEditor workflow tends to get wrong, and writing the file
directly avoids it.

Textures for road, grass, terrain, buildings, and trees are generated in
code, so nothing external is needed to see a complete track. You can
replace them afterwards if you want better-looking surfaces.

## Notes and limits

- Elevation and terrain data come from free services with request
  limits. The tool caches every elevation result to disk
  (`output/elev_cache.json`), so re-exporting the same road doesn't
  re-fetch anything. A very large first export can still hit a limit; if
  that happens, wait a minute and export again — the cached points from
  the first attempt make the retry cheaper.
- Terrain follows 30-metre resolution source data, so a sheer cliff
  renders as a steep slope over a short distance rather than a vertical
  wall.
- How complete the surroundings look depends entirely on how well the
  area is mapped in OpenStreetMap. Well-mapped areas fill in nicely;
  sparsely mapped ones won't.
- Divided roads (separate carriageways sharing one name) can occasionally
  join in a zig-zag. Trim past the affected end with the START/END
  markers if it happens.
- Buildings and trees have no collision. Running into one won't stop the
  car.

## What gets written

```
content/tracks/<name>/
  <name>.kn5          the track model: meshes, textures, spawn points
  models.ini
  map.png             minimap image
  data/map.ini        minimap calibration
  data/surfaces.ini   surface physics
  ui/ui_track.json    track info for the menu / Content Manager
  ui/preview.png
  ui/outline.png
  ai/                 empty; AC writes a lap line here after a hotlap
```

## Files in this folder

- `launch.py` — starts the server and opens the browser
- `server.py` — everything: map lookup, elevation, mesh building, KN5 writing
- `static/index.html` — the map interface
- `output/` — working folder, including the elevation cache
