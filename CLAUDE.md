# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Kolbe: a Bevy 0.19 (Rust) desktop simulation of a peer-to-peer autonomous
drone mesh network over real Swedish terrain (Lantmäteriet elevation data).
Built at EDTH Hamburg 2026. Single binary, `src/main.rs` wires every system
into one `App`.

## Commands

```
cargo run                       # run the app (starts the in-process terrain fetcher; needs .env, see below)
cargo run --features fast-dev   # dev iteration: links Bevy dynamically, skips relinking the engine each rebuild
cargo test                      # run all unit tests (fast — no network)
cargo test <module>::           # e.g. `cargo test networking::` to scope to one module
cargo test -- --ignored --nocapture   # runs live network tests too, e.g. terrain::source::tests::smoke_fetch_stockholm (needs .env + network)
cargo check
```

There is no separate lint/format command configured beyond `cargo fmt` /
`cargo clippy` (standard, not wired into CI). CI (`.github/workflows/rust.yml`)
just runs `cargo test --locked --verbose` on Linux with `mold` as the linker.

### Terrain credentials

Kolbe fetches real elevation data from Lantmäteriet in-process (no external
server). Copy `.env.example` to `.env` and fill in `SECRET` (base64 of
`consumer_key:consumer_secret` from Lantmäteriet's API portal) before
`cargo run` — without it, area selection works but "Generate terrain" fails.
`TERRAIN_VERTICAL_EXAGGERATION` (1–20, default 1) can be set to make lowland
relief more visible.

Note: [README.md](README.md) still describes a separate Python `height_server`
process — that's stale. Terrain fetching, GeoTIFF decoding, and reprojection
were rewritten as an in-process Rust module ([src/terrain/source.rs](src/terrain/source.rs)); there is
no `height_server` directory anymore.

## Architecture

### State machine (`src/main.rs`)

The whole app is one `Bevy` `App` driven by an `AppState`:

```
AreaSelection → LoadingTerrain → Simulation
      ^                               |
      +-------- (reset button) -------+
```

- **AreaSelection** ([src/area.rs](src/area.rs)): a 2D map UI (Sweden outline rasterized by
  [src/sweden_geo.rs](src/sweden_geo.rs)) where the user clicks 3+ points to outline a network
  area; [src/polygon.rs](src/polygon.rs) fits the minimum-area bounding square, then places a
  base location within it.
- **LoadingTerrain** ([src/terrain/mod.rs](src/terrain/mod.rs)): spawns a background `IoTaskPool`
  task ([src/terrain/source.rs](src/terrain/source.rs)) that authenticates, STAC-searches, downloads
  and reprojects Lantmäteriet COG tiles into a height grid; a progress bar
  polls a shared `Arc<Mutex<Progress>>`.
- **Simulation** ([src/world.rs](src/world.rs), [src/base.rs](src/base.rs)): spawns terrain mesh, water plane,
  base, and 12 drones in a ring formation, then runs the mesh-network
  simulation every frame.

Leaving `Simulation` (reset button) despawns everything tagged
`SimulationEntity` and resets the networking resources — see
`teardown_simulation` in [src/main.rs](src/main.rs).

### The real per-frame simulation pipeline

Wired directly as top-level modules in `main.rs` (not through the
`factories/` trait layer — see below). Order matters and is enforced with
`.chain()`:

1. **[src/networking.rs](src/networking.rs)** — the comms protocol only: detecting radio
   links via antenna RSSI, emitting/echoing headers, ranging by round-trip
   timing, gossiping a distance-vector mesh table, and a flooded 3-phase
   reconnection handshake (Request/Accept/Position). Extensive module-level
   doc comments explain the protocol; read them before touching this file.
2. **[src/tracking.rs](src/tracking.rs)** — decides which way each drone's antennas
   physically point, using only comms-derived data (predicted position from
   a peer's last self-reported flight direction, falling back to the mesh
   table) — never a peer's live/omniscient `Transform`. Deliberately
   separate from `networking.rs` (reads its output, never the reverse).
3. **[src/seeking.rs](src/seeking.rs)** — spiral search to reacquire a dropped link, only
   for antenna slots not currently linked.
4. **[src/recovery.rs](src/recovery.rs)** — when a drone was a peer's *sole* mesh
   connection (checked against the local mesh table, not omniscient state)
   and that peer drops, fly back to the last-contact waypoint and hold.
5. **[src/navigation.rs](src/navigation.rs)** — the single authority for actually moving a
   drone (`navigate()`): rate-limited acceleration, asymmetric climb/descend
   caps, braking on approach, rate-limited yaw. Nothing else may write
   position/velocity directly.

`factories::movement::apply_velocity` is the actual integrator that applies
`DroneKinematics::velocity` to `Transform` each frame.

**Currently disabled**: live antenna re-aiming (`tracking::maintain_mesh_antennas`,
`seeking::seek_lost_links`) is not wired into `main.rs`'s system list — antennas
and radar cones stay at their spawn angles. Re-enabling is flagged as a future
PR in code comments (also note the `azimuth_deg` world-frame vs. drone-relative
heading subtraction that needs fixing in both files when it's re-wired).

### The `factories/` module — a *separate*, mostly-unimplemented pluggable AI layer

`src/factories/` defines a `DroneAi` component bundling four traits
(`TrackLogic`, `SeekLogic`, `NetworkLogic`, `MovementLogic`) each with a Rust
stub (mostly `todo!()`) and, behind `--features python`, a `pyo3` bridge to a
same-named Python class. This exists so drone logic could later be swapped
per-drone between Rust and Python — but it is **not** what currently drives
the simulation; the real logic lives in the top-level modules above.
`DroneAi::default()` is inserted on every drone in `world::setup` but its
methods are never called from any system in `main.rs`. Don't assume code in
`factories/{seek,track,network}.rs` is live — check whether a system actually
calls it before relying on its behavior.

### Antenna / RF model ([src/antenna.rs](src/antenna.rs))

Every antenna (drone or base) is a physical model: 3GPP-style parabolic gain
pattern, Friis path loss + linear atmospheric absorption, RSSI = P_tx + G(θ_tx)
+ G(θ_rx) − L(d). A link exists only when `rssi_dbm >= sensitivity_dbm`, i.e.
both antennas happen to be pointed at each other — there is no other
"in range" check anywhere in the sim. `SphericalVec` ([src/spherical.rs](src/spherical.rs)) is
the shared azimuth/elevation/length representation used by both antenna
aiming and ranging results.

Each drone has exactly 3 antennas, meant to lock onto: next ring neighbor,
base, previous ring neighbor (see `tracking::maintain_mesh_antennas`) — so
most drone pairs are deliberately never mutually visible, forcing multi-hop
relay through the mesh table to be exercised.

### Terrain height sampling

`TerrainHeightMap::height_at(x_km, z_km)` (bilinear-sampled, `+Z` = north) is
the single source of ground elevation, read by drone spawn positions, the
grid overlay, contour lines, and the network-area outline. Coordinates
throughout the simulation are **kilometers**, not meters — world-space `Vec3`
positions are in km, velocities in km/s; `recovery.rs`'s `limits_km()` shows
the m/s → km/s conversion pattern for anything using `navigation::FlightLimits`
(which is in real-world m/s units).

### Theming ([src/theme.rs](src/theme.rs))

Catppuccin Mocha (dark) / Latte (light) palette behind a `Theme` resource and
runtime toggle. Entities carry a `ThemeRole` marker component; `apply_theme`
(and `apply_loading_theme` for the loading screen) recolor materials when the
`Theme` resource changes. When spawning new themed geometry, set the initial
color from `theme.palette()` *and* attach the matching `ThemeRole` — both
systems query by that marker to know what to recolor later, and forgetting
the marker leaves an entity frozen at its spawn-time color after a toggle.

### Bevy version quirks

Uses Bevy 0.19 with the newer observer-based picking API (`On<Pointer<Click>>`,
`.observe(...)`) rather than the older event-reader pattern, and the newer
`FontSize`/`TextFont` UI text API. `apply_theme`/`apply_loading_theme` had a
query-conflict panic history (see git log) — be careful with overlapping
mutable queries across systems that both write `BackgroundColor`/materials in
the same schedule.
