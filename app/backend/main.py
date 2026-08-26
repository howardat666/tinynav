"""
TinyNav backend — FastAPI + uvicorn.

Usage:
    cd /tinynav
    TINYNAV_DB_PATH=/tinynav/tinynav_db uv run uvicorn app.backend.main:app --host 0.0.0.0 --port 8000

It can also serve the Flutter web bundle itself, so that one process on one port
is the whole app. See SERVING THE WEB UI below.
"""
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.staticfiles import StaticFiles

from .manager_client import BACKEND_ROLE, is_display_role
from .state import runner
from .routers import action, bag, device, files, logs, nav, sensor
from .routers import map as map_router
from .routers import poi
from .routers import proxy
from . import ws


@asynccontextmanager
async def lifespan(app: FastAPI):
    runner.start()
    yield
    runner.stop()


app = FastAPI(title=f'TinyNav API ({BACKEND_ROLE})', version='0.1.0', lifespan=lifespan)

# StaticFiles does not compress, and the Flutter bundle's first load is main.dart.js
# 3.04 MB + canvaskit.wasm 7.16 MB = 10.2 MB, which gzip takes to 3.8 MB (measured).
# On a link that has been seen dropping to CCK_1M that is the difference between the page
# opening and not -- 2026-08-20 a frontend redeploy invalidated the browser cache and the
# page stopped loading, with repeated 206 Partial Content on exactly those two files.
# minimum_size keeps it off the small JSON the API returns every cycle.
app.add_middleware(GZipMiddleware, minimum_size=2048)

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)

app.include_router(device.router, prefix='/device')
app.include_router(sensor.router)
app.include_router(ws.router)

if is_display_role():
    # Jetson display backend owns topic subscriptions/WebSockets and forwards
    # manager-owned HTTP routes to the insight9 manager backend.
    app.include_router(proxy.router)
else:
    app.include_router(bag.router, prefix='/bag')
    app.include_router(map_router.router, prefix='/map')
    app.include_router(poi.router)
    app.include_router(nav.router, prefix='/nav')
    app.include_router(files.router)
    app.include_router(logs.router)
    app.include_router(action.router)


# SERVING THE WEB UI
# ------------------
# Mounted last and only last: a mount at '/' matches every path the routers above
# did not claim, so registering it any earlier would shadow the API.
#
# Serving the bundle from the same origin as the API is the point, not a
# convenience. The frontend derives its endpoints from a stored device IP that
# defaults to 169.254.10.1 and appends :8000, so a page fetched from
# http://169.254.10.1:8000/ talks back to exactly where it came from -- no IP to
# type in, no CORS preflight, and one uvicorn instead of a second static server.
# That matters here because the X5 inside the camera is the only computer in the
# system; the laptop is a browser and nothing else.
#
# Absent bundle is not an error. In the docker dev flow the frontend runs under
# `flutter run -d chrome` on a different port and this directory does not exist,
# so mounting is skipped and the API behaves exactly as before.
_DEFAULT_WEB_ROOT = Path(__file__).resolve().parents[1] / 'frontend' / 'build' / 'web'
WEB_ROOT = Path(os.environ.get('TINYNAV_WEB_ROOT', _DEFAULT_WEB_ROOT))

if (WEB_ROOT / 'index.html').is_file():
    # html=True makes a directory path serve its index.html, so '/' returns the
    # app. It is not an SPA fallback: an unknown path still 404s. That is correct
    # here because this frontend keeps all its state in widgets and never puts a
    # route in the URL -- there are no deep links to preserve. Should it ever grow
    # URL routing, this mount would need a catch-all returning index.html instead.
    app.mount('/', StaticFiles(directory=str(WEB_ROOT), html=True), name='web')
    print(f'[backend] serving web UI from {WEB_ROOT}', flush=True)
else:
    print(f'[backend] no web bundle at {WEB_ROOT}, API only', flush=True)
