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
