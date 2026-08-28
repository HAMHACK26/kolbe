"""Lantmateriet elevation proxy, adapted from dies_irae/height_server (MIT)."""

from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
import hashlib
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rasterio
import rasterio.warp
import requests
from dotenv import load_dotenv
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.merge import merge
from rasterio.transform import from_bounds

load_dotenv()

TOKEN_URL = os.environ.get(
    "STAC_TOKEN_URL", "https://apimanager.lantmateriet.se/oauth2/token"
)
SEARCH_URL = os.environ.get(
    "STAC_SEARCH_URL", "https://api.lantmateriet.se/stac-hojd/v1/search"
)
STAC_COLLECTION = os.environ.get("STAC_COLLECTION", "dtm-cog")
SECRET = os.environ.get("SECRET", "")
RADIUS_KM = float(os.environ.get("RADIUS_KM", "10"))
OUTPUT_SIZE = int(os.environ.get("OUTPUT_SIZE", "129"))
TIMEOUT = int(os.environ.get("DEFAULT_TIMEOUT", "60"))
PORT = int(os.environ.get("PORT", "8000"))
DOWNLOAD_WORKERS = int(os.environ.get("DOWNLOAD_WORKERS", "8"))
PROCESSING_THREADS = int(os.environ.get("PROCESSING_THREADS", "4"))
STREAM_COG = os.environ.get("STREAM_COG", "true").lower() not in ("0", "false", "no")
CACHE_DIR = Path(os.environ.get("CACHE_DIR", ".cache"))
TILE_CACHE_DIR = CACHE_DIR / "tiles"
WGS84 = CRS.from_epsg(4326)

if not SECRET:
    raise RuntimeError("SECRET is missing; copy .env.example to .env and configure it")
if not 0.1 <= RADIUS_KM <= 50:
    raise RuntimeError("RADIUS_KM must be between 0.1 and 50")
if not 2 <= OUTPUT_SIZE <= 1025:
    raise RuntimeError("OUTPUT_SIZE must be between 2 and 1025")
if not 1 <= DOWNLOAD_WORKERS <= 16:
    raise RuntimeError("DOWNLOAD_WORKERS must be between 1 and 16")
if not 1 <= PROCESSING_THREADS <= 16:
    raise RuntimeError("PROCESSING_THREADS must be between 1 and 16")
CACHE_DIR.mkdir(parents=True, exist_ok=True)
TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)

_token = ""
_token_expires_at = 0.0
_token_lock = threading.Lock()
_progress_lock = threading.Lock()
_progress = {"phase": "idle", "done": 0, "total": 0, "current": ""}
_cache_lock = threading.Lock()


def set_progress(phase: str, done: int = 0, total: int = 0, current: str = ""):
    with _progress_lock:
        _progress.update(phase=phase, done=done, total=total, current=current)


def progress_text() -> bytes:
    with _progress_lock:
        values = _progress.copy()
    # A deliberately tiny line protocol avoids adding JSON dependencies to the client.
    return (
        f"phase={values['phase']}\n"
        f"done={values['done']}\n"
        f"total={values['total']}\n"
        f"current={values['current']}\n"
    ).encode("utf-8")


def cache_path(lat: float, lon: float) -> Path:
    key = f"{STAC_COLLECTION}_{lat:.5f}_{lon:.5f}_{RADIUS_KM:g}_{OUTPUT_SIZE}"
    return CACHE_DIR / f"terrain_{key}.npz"


def load_cached(lat: float, lon: float):
    path = cache_path(lat, lon)
    if not path.exists():
        return None
    try:
        with _cache_lock, np.load(path) as cached:
            array = cached["array"].astype("<f2", copy=False)
            bbox = tuple(float(value) for value in cached["bbox"])
        if array.shape != (OUTPUT_SIZE, OUTPUT_SIZE):
            raise ValueError("cached grid has unexpected dimensions")
        set_progress("Loaded terrain from cache", done=1, total=1)
        return array, bbox
    except (OSError, ValueError, KeyError):
        try:
            path.unlink()
        except OSError:
            pass
        return None


def save_cached(lat: float, lon: float, array, bbox):
    destination = cache_path(lat, lon)
    temporary = destination.with_suffix(".tmp.npz")
    with _cache_lock:
        np.savez_compressed(temporary, array=array, bbox=np.asarray(bbox))
        os.replace(temporary, destination)


def access_token() -> str:
    global _token, _token_expires_at
    with _token_lock:
        if _token and time.time() < _token_expires_at - 30:
            return _token
        set_progress("Authenticating with Lantmateriet")
        response = requests.post(
            TOKEN_URL,
            headers={"Authorization": f"Basic {SECRET}"},
            data={"grant_type": "client_credentials"},
            timeout=TIMEOUT,
        )
        response.raise_for_status()
        data = response.json()
        _token = data["access_token"]
        _token_expires_at = time.time() + int(data.get("expires_in", 3600))
        return _token


def bbox_from_center(lat: float, lon: float):
    if not 54.0 <= lat <= 70.0 or not 10.0 <= lon <= 25.0:
        raise ValueError("Coordinates must be inside Sweden")
    lat_delta = RADIUS_KM / 111.32
    lon_delta = RADIUS_KM / (111.32 * math.cos(math.radians(lat)))
    return lon - lon_delta, lat - lat_delta, lon + lon_delta, lat + lat_delta


def stac_features(lat: float, lon: float):
    set_progress("Searching elevation catalogue")
    bbox = bbox_from_center(lat, lon)
    params = {
        "bbox": ",".join(map(str, bbox)),
        "collections": STAC_COLLECTION,
        "limit": 100,
    }
    headers = {"Authorization": f"Bearer {access_token()}"}
    features = []
    url = SEARCH_URL
    while url:
        response = requests.get(url, headers=headers, params=params, timeout=TIMEOUT)
        response.raise_for_status()
        data = response.json()
        features.extend(data.get("features", []))
        url = next(
            (link.get("href") for link in data.get("links", []) if link.get("rel") == "next"),
            None,
        )
        params = {}
    return features, bbox


def asset_urls(features):
    urls = []
    for feature in features:
        for asset in feature.get("assets", {}).values():
            href = asset.get("href", "")
            kind = asset.get("type", "").lower()
            path = urlparse(href).path.lower()
            if href and ("tiff" in kind or path.endswith((".tif", ".tiff"))):
                urls.append(href)
    return list(dict.fromkeys(urls))


def tile_cache_path(url: str) -> Path:
    parsed = urlparse(url)
    # Ignore temporary query parameters so refreshed STAC links still reuse
    # the same cached elevation asset.
    identity = f"{parsed.netloc}{parsed.path}".encode("utf-8")
    return TILE_CACHE_DIR / f"{hashlib.sha256(identity).hexdigest()}.tif"


def download(url: str) -> str:
    filename = url.rsplit("/", 1)[-1].split("?", 1)[0]
    with _progress_lock:
        _progress["current"] = filename
    destination = tile_cache_path(url)
    if destination.exists() and destination.stat().st_size > 0:
        with _progress_lock:
            _progress["done"] += 1
        return str(destination)

    partial = destination.with_suffix(".tif.part")
    offset = partial.stat().st_size if partial.exists() else 0
    headers = {"Authorization": f"Bearer {access_token()}"}
    if offset:
        headers["Range"] = f"bytes={offset}-"
    response = requests.get(
        url,
        headers=headers,
        stream=True,
        timeout=180,
    )
    response.raise_for_status()
    mode = "ab" if offset and response.status_code == 206 else "wb"
    with open(partial, mode) as output:
        for chunk in response.iter_content(1024 * 1024):
            if chunk:
                output.write(chunk)
    os.replace(partial, destination)
    with _progress_lock:
        _progress["done"] += 1
    return str(destination)


def build_height_array(sources, bbox, cog_token=None):
    datasets = []
    try:
        phase = "Reading cloud terrain" if cog_token else "Opening cached tiles"
        set_progress(phase, total=len(sources))
        for source in sources:
            datasets.append(rasterio.open(source))
            with _progress_lock:
                _progress["done"] += 1
                _progress["current"] = Path(urlparse(source).path).name
        native_crs = datasets[0].crs
        left, bottom, right, top = bbox
        xs, ys = rasterio.warp.transform(
            WGS84,
            native_crs,
            [left, right, left, right],
            [bottom, bottom, top, top],
        )
        set_progress("Building terrain grid", total=len(sources), done=len(sources))
        # Merge near the final mesh resolution. Merging at the source raster's
        # native resolution first can consume gigabytes for a 20 x 20 km area.
        merge_resolution = (
            (max(xs) - min(xs)) / OUTPUT_SIZE,
            (max(ys) - min(ys)) / OUTPUT_SIZE,
        )
        source, source_transform = merge(
            datasets,
            bounds=(min(xs), min(ys), max(xs), max(ys)),
            nodata=np.nan,
            res=merge_resolution,
            resampling=Resampling.bilinear,
        )
        result = np.full((OUTPUT_SIZE, OUTPUT_SIZE), np.nan, dtype=np.float32)
        set_progress("Resampling terrain", done=0, total=1)
        rasterio.warp.reproject(
            source=source[0].astype(np.float32, copy=False),
            destination=result,
            src_transform=source_transform,
            src_crs=native_crs,
            dst_transform=from_bounds(*bbox, OUTPUT_SIZE, OUTPUT_SIZE),
            dst_crs=WGS84,
            resampling=Resampling.bilinear,
            num_threads=PROCESSING_THREADS,
            src_nodata=np.nan,
            dst_nodata=np.nan,
        )
        if not np.any(np.isfinite(result)):
            raise RuntimeError("Elevation raster contains no valid samples")
        minimum = float(np.nanmin(result))
        result = np.nan_to_num(result - minimum, nan=0.0)
        return result.astype("<f2")
    finally:
        for dataset in datasets:
            dataset.close()


def load_height_array(lat: float, lon: float):
    set_progress("Starting terrain request")
    cached = load_cached(lat, lon)
    if cached is not None:
        return cached
    features, bbox = stac_features(lat, lon)
    urls = asset_urls(features)
    if not urls:
        raise RuntimeError("No elevation raster covers the selected area")

    height = None
    if STREAM_COG:
        token = access_token()
        try:
            # The product is COG, so GDAL can fetch only the overview/raster
            # blocks needed for our small output grid instead of whole TIFFs.
            with rasterio.Env(
                GDAL_HTTP_HEADERS=f"Authorization: Bearer {token}",
                GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR",
                CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif,.tiff",
            ):
                height = build_height_array(urls, bbox, cog_token=token)
        except rasterio.errors.RasterioError:
            set_progress("Cloud reads unavailable; downloading tiles")

    if height is None:
        set_progress("Downloading elevation tiles", total=len(urls))
        with ThreadPoolExecutor(max_workers=min(len(urls), DOWNLOAD_WORKERS)) as pool:
            paths = list(pool.map(download, urls))
        height = build_height_array(paths, bbox)

    set_progress("Saving terrain cache", done=0, total=1)
    save_cached(lat, lon, height, bbox)
    set_progress("Terrain ready", done=1, total=1)
    return height, bbox


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/health":
            self.respond(b"ok", "text/plain")
            return
        if parsed.path == "/progress":
            self.respond(progress_text(), "text/plain; charset=utf-8")
            return
        if parsed.path != "/fetch":
            self.error(404, "Use /fetch?lat=...&lon=...")
            return
        try:
            query = parse_qs(parsed.query)
            lat = float(query["lat"][0])
            lon = float(query["lon"][0])
            array, bbox = load_height_array(lat, lon)
            payload = array.tobytes(order="C")
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("X-Width", str(OUTPUT_SIZE))
            self.send_header("X-Height", str(OUTPUT_SIZE))
            self.send_header("X-Dtype", "float16-le")
            self.send_header("X-BBox", ",".join(map(str, bbox)))
            self.end_headers()
            self.wfile.write(payload)
        except (KeyError, ValueError) as exc:
            self.error(400, str(exc))
        except Exception as exc:
            set_progress("Terrain request failed")
            self.error(502, str(exc))

    def respond(self, payload: bytes, content_type: str):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def error(self, status: int, message: str):
        payload = (message + "\n").encode("utf-8", errors="replace")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Height server listening on http://127.0.0.1:{PORT}")
    server.serve_forever()
