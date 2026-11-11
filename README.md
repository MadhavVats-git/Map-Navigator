# Map Navigator (C++)

A route-finding app for a minor project. The C++ backend exposes a REST API and also serves the web frontend, so one process is the whole app.

## Structure
- `backend/main.cpp` – C++17 HTTP server (no external libraries), graph of 17 Delhi landmarks, Dijkstra and A* routing
- `frontend/index.html` – canvas map UI that calls the API with `fetch`
- `Dockerfile`, `Makefile` – build and deploy

## Run locally
```
make
./navigator            # PORT=8080 by default; set PORT to change
```
Open http://localhost:8080

## API
| Endpoint | Description |
|---|---|
| `GET /api/health` | Status, node and edge counts |
| `GET /api/places` | All places with id, name, lat, lon |
| `GET /api/graph` | Nodes and roads (km, speed) |
| `GET /api/route?from=0&to=8&algo=astar&mode=distance` | Route. `algo`: `astar` or `dijkstra`. `mode`: `distance` or `time` |

Example:
```
curl "http://localhost:8080/api/route?from=0&to=8&algo=dijkstra&mode=time"
```

## Deploy
**Docker (any host, e.g. a VPS):**
```
docker build -t map-navigator .
docker run -p 8080:8080 map-navigator
```

**Render / Railway / Fly.io:** create a new web service from this folder's Git repo, choose the Dockerfile, and the platform sets `PORT` for you.

**Frontend on a different host:** open the frontend with `?api=https://your-api-host`; the API already sends CORS headers.

## How routing works
Roads are an undirected weighted graph. Road length is the straight-line (haversine) distance times 1.3. Dijkstra explores outward by cost; A* adds a haversine heuristic, which is admissible, so both return the same optimal route but A* usually explores fewer places (see `nodes_explored`).
