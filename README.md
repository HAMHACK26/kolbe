# kolbe
A peer-to-peer autonomous 3d drone mesh network, made in EDTH Hamburg 2026

## Lantmateriet terrain

Keep the API credentials in `height_server/.env`. This file is gitignored; only
`height_server/.env.example` is committed.

1. Create a Python virtual environment in `height_server`.
2. Install `height_server/requirements.txt`.
3. Copy `.env.example` to `.env` and fill in `SECRET`.
4. Run Kolbe with `cargo run`; it starts the bundled height server automatically.

Kolbe connects to `http://127.0.0.1:8000` by default. Set `HEIGHT_SERVER_URL`
before starting Kolbe when the service runs elsewhere.

Terrain is displayed with 5x vertical exaggeration so Swedish lowland relief is
visible at the 20 km map scale. Set `TERRAIN_VERTICAL_EXAGGERATION` before
`cargo run` to choose a value from 1 (true scale) through 20.

## Trees

Tree heights are requested during the terrain background load from
Skogsstyrelsen's public `Tradhojd_3_1` ImageServer. Vegetation failure is
non-fatal: terrain still opens and the error is logged. Detected canopy maxima
are placed on the terrain and rendered with shared meshes and materials.

- `VEGETATION_ENABLED` — set to `false` to skip trees.
- `TREE_MIN_HEIGHT_M` — minimum height (default `5`).
- `TREE_MIN_SPACING_M` — minimum detected-tree spacing (default `8`).
- `TREE_MAX_COUNT` — deterministic tallest-first limit (default `10000`).
- `TREE_LOD_DISTANCE_KM` — maximum rendering distance (default `40`).
- `TREE_RASTER_RESOLUTION_M` — requested resolution (default `5`, range `2..100`).
- `TREE_HEIGHT_SERVICE_URL` — overrides the public ImageServer endpoint.
- `TREE_HEIGHT_RASTER_PATH` — uses an offline text raster instead.

The offline format starts with `width height pixel_size_m`, followed by exactly
`width * height` signed heights in decimetres. Row zero is the north edge.
