# AC Road Tool

Turn any real road into a drivable Assetto Corsa track. You search for a road,
pick the stretch you want, and the tool builds a finished track package with
real elevation, terrain, buildings, trees, side roads and barriers.

There are two ways to run it. **Automatic mode** works anywhere in the world
using free online elevation data. **Manual mode** uses a high resolution LiDAR
survey file that you download yourself, which is dramatically more accurate but
only available for areas that have been surveyed.

---

## Getting started

You need Python 3.9 or newer. Everything else installs itself.

```
python launch.py
```

That checks for numpy, scipy, pyproj and pillow, installs anything missing,
starts the server on port 8743 and opens your browser at
`http://localhost:8743`.

If you would rather run the server on its own, `python server.py` does the same
thing without opening a browser.

---

## Automatic mode

This is the default. No setup, no downloads, works for any road that exists in
OpenStreetMap.

The badge at the top of the sidebar will read **ELEVATION: AUTOMATIC**.

### 1. Find your road

Type a road name into the search box. Adding the suburb or region helps a lot,
so "Gorge Road, Adelaide" beats "Gorge Road" on its own. Results appear
underneath. Click the one you want.

The tool then downloads the road's geometry and draws it on the map. Roads in
OpenStreetMap are often split into dozens of separate pieces, so behind the
scenes it stitches those pieces back into one continuous route.

### 2. Choose your section

Very few people want an entire road, so nothing is built until you say which
part you want.

Drag the **START** and **END** markers along the road to bracket the stretch you
want, then press **Confirm Section**. Preview and Export stay locked until you
do. If you move a marker afterwards the section unlocks again, so you just press
Confirm once more.

### 3. Set the parameters

- **Road Width**: total carriageway width in metres. 8m suits a typical two lane
  road, 6m a narrow country lane, 12m or more for a highway.
- **GPS Smoothing**: how much to smooth the road's *shape*. OpenStreetMap
  geometry is traced by hand and often slightly jagged. Around 30% is a good
  default. Lower it if the tool is rounding off corners you want to keep, raise
  it if the road feels twitchy at speed.
- **Max Road Grade**: leave this on **Auto**. It works out how steep your road
  genuinely gets from its own long range trend, then treats anything much
  steeper as a data error and smooths it out. If your road has genuinely
  dramatic dips that are being flattened, untick Auto and slide it up to 35%,
  which effectively turns the correction off.
- **Steady climb**: tick this only if you know the road never reverses
  direction. It forces the elevation to rise (or fall) consistently, which wipes
  out phantom dips caused by bad data. It is a strong assumption, so only use it
  when you are sure.

### 4. Preview

**Preview Mesh Stats** builds the road without exporting it and reports length,
corner count, total climb and elevation range, plus an elevation profile chart.

The chart is worth understanding, because it is how you tell good data from bad:

- **Gold line**: the final road as it will be built.
- **Red line**: the raw elevation data before any correction. If a strange hill
  appears here too, the problem is in the source data, not the processing.
- **Blue line**: an independent second opinion from a different satellite
  mission. Where blue agrees with red, the feature is probably real terrain.
  Where they disagree, at least one of them is wrong.
- **Grey band**: how far the elevation could be off at that point. It widens
  where the road is cut into a hillside, because a small horizontal error
  becomes a large vertical one on steep ground.
- **Text underneath**: typical confidence in metres, and how much of the route
  sits on steep sidehill.

Adjust parameters and preview again as often as you like. Previews are quick
because everything is cached.

### 5. Export and install

Give the track a name, then press **Export AC Package**.

If you paste your Assetto Corsa folder path into the install box (something like
`C:\Program Files (x86)\Steam\steamapps\common\assettocorsa`) the tool installs
the track directly. Otherwise you get a zip to download, which you extract into
that same Assetto Corsa root folder so the track lands in `content/tracks/`.

Launch it in **Hotlap** or **Practice** mode. Race mode needs an AI line, which
AC will generate for you after you drive the road once in Hotlap.

---

## Manual mode (LiDAR)

Automatic mode relies on satellite radar data at roughly 30 metre resolution,
which is fine on open ground but struggles in two situations: it measures tree
canopy rather than the ground underneath, and on a road cut into a hillside a
small horizontal error turns into a large vertical one. That is where phantom
dips and bumps come from.

Manual mode replaces that with an airborne LiDAR survey, typically 1 metre
resolution with vertical accuracy in the 5 to 15 centimetre range. It removes
essentially the entire problem.

You still search for and select the road exactly as in automatic mode. The only
thing the LiDAR file changes is where the heights come from.

### 1. Install rasterio

```
pip install rasterio
```

This one is not installed automatically, since it is only needed for manual
mode.

### 2. Download the elevation data

Australian users can get this free from Geoscience Australia's ELVIS portal:

1. Go to `elevation.fsdf.org.au` and accept the terms.
2. Zoom to your road. Shaded areas have LiDAR coverage.
3. Open the menu and choose **Order Data**, then draw a box around your road. A
   tight box keeps the download small.
4. Pick a **DEM** product, ideally 1 Metre DEM. Make sure it says DEM (bare
   earth ground surface) rather than DSM, which includes trees and buildings.
5. Enter your email address. A download link arrives shortly afterwards.

If only 50cm data is offered, that is fine too. It is not more *accurate* than
the 1m version, just a finer grid, and this tool samples along the road every
10 metres so the extra detail is not really used. Either works.

Outside Australia, look for your national mapping agency's open LiDAR programme.
Any georeferenced elevation GeoTIFF will work, since the tool reads the
projection from the file itself.

### 3. Create the dem folder

This is the step people miss. The tool does not create this folder for you, and
if it is not there the tool simply stays in automatic mode without complaining.

Make a folder called exactly `dem`, sitting next to `server.py`:

```
your-folder/
  server.py
  index.html
  launch.py
  dem/          <-- create this yourself
```

From a terminal in that folder:

```
mkdir dem
```

The name is lowercase and the location matters. A `dem` folder inside `output`,
or one sitting beside the folder rather than inside it, will not be found.

### 4. Add the files

Unzip your download and put the `.tif` files straight into `dem`. There is no
need to keep any subfolders the zip came with, and you can rename the files to
whatever you like.

```
your-folder/
  server.py
  dem/
    your-lidar-tile.tif
    another-tile.tif
```

Multiple tiles are fine, and the tool loads all of them. If two tiles overlap,
the finer resolution one wins.

Restart the server. The console confirms what it found:

```
[dem] loaded your-lidar-tile.tif: 8000x8000 px @ 1.0 m/px (EPSG:28354)
```

### 5. Confirm it is being used

Three things tell you manual mode is active:

- The sidebar badge turns gold and reads **ELEVATION: MANUAL**, with the file
  count and resolution.
- A dashed gold rectangle appears on the map showing exactly what your file
  covers. Your road needs to be inside it.
- After a preview or export the badge adds the coverage for your selected road,
  for example "100% of selected road covered".

If part of your road falls outside the file you get a warning, and that part
quietly falls back to online data with the vertical datums matched so there is
no step at the boundary.

### 6. What changes automatically

When LiDAR covers more than 90% of your selected road, the tool reconfigures
itself, because most of its correction machinery exists to fight errors that no
longer exist:

- **Grade limiting turns off.** The Max Road Grade field displays `OFF · LiDAR`.
  A steep pinch in LiDAR data is a real steep pinch.
- **Elevation smoothing drops** from 30 metres to 8 metres, so real crests and
  compressions survive into the game instead of being averaged away.
- **Corridor sampling narrows** from 15 metres to 3 metres either side, because
  LiDAR resolves the actual road surface and a wide sample would average the
  road with the cut batters beside it.
- **Canopy correction switches off**, since bare earth LiDAR has no trees in it
  to remove.

A note on the panel: **GPS Smoothing stays active in both modes**. That slider
smooths the road's *shape*, which comes from hand traced OpenStreetMap geometry
regardless of where the heights come from. It never touches elevation.

Two things stay on in both modes. Bridges and tunnels are still levelled, since
bare earth LiDAR removes bridge decks too and would otherwise send you diving
into the gully underneath. A light spike filter also stays, which is harmless on
clean data.

---

## How it works

### Road geometry

Search goes through Nominatim, which turns a name into an OpenStreetMap
identifier. The tool then pulls every segment of that road from Overpass and
stitches them into one route by matching endpoints, since a single named road is
usually stored as many separate ways. Your start and end markers trim that route
down to the section you actually want.

The centreline is then resampled at even intervals and smoothed with a spline,
which is what the GPS Smoothing slider controls.

### Elevation

Heights come from AWS Terrain Tiles, which encode elevation as colour values in
PNG map tiles. The tool picks the finest zoom level that fits within its tile
budget, so short roads get full detail and long ones degrade gracefully.

Rather than sampling a single point on the centreline, it samples a cross
section at each station and fits a curve across it, evaluated at the centre.
That averages out noise without biasing the result on a hillside or in a valley.
The steepness of that cross section also tells the tool how trustworthy each
point is, which is what the grey uncertainty band on the chart shows.

The profile then goes through several passes: a spike filter, the grade limiter
described above, levelling for anything tagged as a bridge, tunnel, embankment
or cutting (all of which carry the road above or below the natural ground), the
optional steady climb fit, and finally a Gaussian smooth.

### Terrain

A grid of ground points is built out to 90 metres either side of the road,
sampled from the same elevation source and blended into the road height near the
edges so the surface meets the carriageway cleanly. A short blend distance is
used deliberately, so real cliffs and cuttings beside the road keep their shape
instead of being flattened into gentle verges.

In automatic mode there is a canopy correction step. Radar elevation data
measures treetops, so a patch of forest reads as a hill that is not there. Using
ESA WorldCover land classification, tree covered grid points are lowered by a
typical canopy height, with a floor that prevents it carving craters where the
data actually saw the ground. Bare ground is never touched, so real hills
survive.

### Environment

Everything within 60 metres of the road comes from a single Overpass query:
buildings, other roads, waterways, barriers and vegetation areas.

- **Buildings** are extruded from their footprints. Rectangular low footprints
  get a proper pitched gable roof, complex ones get a flat roof triangulated so
  it does not fold over itself. Walls alternate between weatherboard and brick.
- **Trees** come from mapped tree nodes plus a density based scatter across
  woodland areas. Each one is randomly sized and rotated, in two species, so
  rows do not look copy pasted. Any tree that would stand on the carriageway is
  removed.
- **Side roads** are draped over the terrain, clipped where they would overlap
  your drivable road, and extended slightly underneath its edge so junctions
  join up with no gap.
- **Waterways** work the same way, tucked under the road where they cross so
  they read as culverts.
- **Barriers** become fences, walls, hedges or guard rails. Guard rails are
  built as a floating steel band on posts rather than a solid wall.

None of this has collision. Only the road surface does, which is deliberate: it
means the environment can never throw your car around.

### Building the track

The mesh is written directly as a KN5, which is Assetto Corsa's native model
format, with generated textures and materials packed inside. Meshes whose names
begin with a digit are physical surfaces in AC and get collision, so the road is
named `1ROAD` while everything decorative is prefixed `ENV_`.

Alongside the model, the package includes the track configuration, surface
definitions, spawn points (pit, start line, hotlap and time attack), a preview
image, an outline map, and an optional OBJ export under `extras/` if you want to
edit the track in Blender.

### Caching

Everything downloaded is cached under `output/`: elevation tiles, land cover,
road geometry and surroundings. Re-exporting the same road costs no network
requests at all, which matters because the free OpenStreetMap servers are
frequently busy. If a fetch fails the tool retries across five different mirrors
with increasing delays, and it will tell you plainly if it genuinely could not
get the data rather than quietly building an empty looking track.

You can delete the `output` folder any time to clear the cache.

---

## Data sources and APIs

Everything here is free and needs no account or API key.

| Service | Used for | Endpoint |
|---|---|---|
| **Nominatim** | Road name search | `nominatim.openstreetmap.org` |
| **Overpass API** | Road geometry, buildings, trees, side roads, waterways, barriers | `overpass-api.de` and four mirrors: `overpass.kumi.systems`, `lz4.overpass-api.de`, `z.overpass-api.de`, `overpass.private.coffee` |
| **AWS Terrain Tiles** | Elevation (automatic mode). Terrarium encoded PNG tiles, sourced from SRTM and other public datasets | `s3.amazonaws.com/elevation-tiles-prod` |
| **Open-Meteo Elevation** | Copernicus GLO-90 elevation, used as the independent cross check on the profile chart | `api.open-meteo.com/v1/elevation` |
| **ESA WorldCover** | Land classification for texturing, tree density and canopy correction. Version 200, 2021 | `esa-worldcover.s3.eu-central-1.amazonaws.com` |
| **Geoscience Australia ELVIS** | LiDAR DEM downloads for manual mode. Not called by the tool, you download from it yourself | `elevation.fsdf.org.au` |

Map data from Nominatim and Overpass is © OpenStreetMap contributors, available
under the Open Database License.

---

## Troubleshooting

**Search finds nothing.** Add the suburb, city or state to the query. Nominatim
matches names literally.

**"OpenStreetMap servers are busy".** The public Overpass mirrors get
overloaded, usually briefly. Wait a minute and export again. The second attempt
reuses cached data, so it is much faster.

**The elevation profile has a dip that is not real.** Check whether the red raw
line has it too. If it does, the source data is wrong. Try lowering Max Road
Grade manually, or tick Steady climb if the road genuinely only climbs. LiDAR
via manual mode fixes this properly.

**Manual mode is not activating.** The most common cause is a missing `dem`
folder, which you have to create yourself since the tool does not make it. Check
that it is named `dem` in lowercase, that it sits next to `server.py`, that the
`.tif` files are directly inside it, that rasterio is installed, and that you
restarted the server afterwards. The console prints what it loaded on startup,
so if you see no `[dem]` lines at all it did not find the folder.

**The track will not start in Race mode.** Use Hotlap or Practice. Race needs an
AI line, which AC generates after one Hotlap run.
