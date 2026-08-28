# Kolbe height server

Adapted from the MIT-licensed `height_server` in
[`21st-centuryman/dies_irae`](https://github.com/21st-centuryman/dies_irae/tree/main/height_server).

```powershell
cd height_server
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
Copy-Item .env.example .env
# Fill in SECRET in .env, then return to the repository root and run cargo run.
```

PowerShell activation is not required. To run only the server for debugging,
use `.\.venv\Scripts\python.exe height.py` from this directory.

Kolbe requests `GET /fetch?lat=...&lon=...`. The response is a row-major,
little-endian float16 height grid in metres. Dimensions and bounds are returned
in the `X-Width`, `X-Height`, `X-Dtype`, and `X-BBox` headers.

Processed terrain grids are cached in `height_server/.cache` using their centre,
radius, and output resolution. By default the server reads only the required
blocks from Lantmateriet's cloud-optimized GeoTIFFs. If a server does not allow
ranged COG access, it automatically falls back to downloading and caching whole
TIFF tiles. Interrupted downloads resume when the remote server supports HTTP
range requests. Tune `DOWNLOAD_WORKERS` and Rasterio's `PROCESSING_THREADS` in
`.env` when needed. Set `STREAM_COG=false` to force full downloads. The cache is
ignored by Git and can be deleted whenever disk space needs to be reclaimed.
The STAC query is restricted to the `dtm-cog` ground-elevation collection so
forest point clouds and unrelated height products are never downloaded.
