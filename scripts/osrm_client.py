"""Minimal client for an OSRM server (snapping, travel-time tables, route shapes).

The server URL comes from OSRM_URL, set in `.env` at the repo root (see `.env.example`).
Every answer is checked before use: the status code, OSRM's own "code" field, and the shape of the result.
"""
import logging
import os

import numpy as np
import requests

from common import load_env

LOG = logging.getLogger(__name__)
BLOCK = 100  # OSRM's default --max-table-size; larger tables are requested in blocks of this many points
_session = requests.Session()


def osrm_url():
    """Return OSRM_URL from the environment or `.env`, without a trailing slash."""
    load_env()
    url = os.environ.get("OSRM_URL")
    if not url:
        raise EnvironmentError("OSRM_URL is not set. Copy .env.example to .env and point it at an OSRM server.")
    return url.rstrip("/")


def _get(service, coords, **params):
    """Call one OSRM service for a list of (lon, lat) points and return the parsed JSON answer."""
    path = ";".join(f"{lon:.6f},{lat:.6f}" for lon, lat in coords)
    url = f"{osrm_url()}/{service}/v1/driving/{path}"
    resp = _session.get(url, params=params, timeout=120)
    if resp.status_code != 200:
        raise RuntimeError(f"OSRM {service} returned HTTP {resp.status_code}: {resp.text[:200]}")
    answer = resp.json()
    if answer.get("code") != "Ok":
        raise RuntimeError(f"OSRM {service} answered {answer.get('code')}: {answer.get('message')}")
    return answer


def nearest(lon, lat, exclude=None):
    """Snap one point to the road network.

    :return: (snapped lon, snapped lat, snap distance in m, OSRM hint that pins this snap in later calls)
    """
    params = {"number": 1}
    if exclude:
        params["exclude"] = exclude
    wp = _get("nearest", [(lon, lat)], **params)["waypoints"][0]
    return wp["location"][0], wp["location"][1], wp["distance"], wp["hint"]


def table(coords, hints=None):
    """Full travel-time [s] and distance [m] matrices between all points.

    The matrix is filled in BLOCK x BLOCK pieces so it also works on servers with the default table size limit.
    Pairs OSRM cannot route come back as NaN; the caller decides what to do with them.
    """
    n = len(coords)
    tt = np.full((n, n), np.nan)
    dist = np.full((n, n), np.nan)
    for i0 in range(0, n, BLOCK):
        for j0 in range(0, n, BLOCK):
            src = list(range(i0, min(i0 + BLOCK, n)))
            dst = list(range(j0, min(j0 + BLOCK, n)))
            idx = src + [j for j in dst if j not in src]  # the points sent in this request
            params = {
                "sources": ";".join(str(idx.index(i)) for i in src),
                "destinations": ";".join(str(idx.index(j)) for j in dst),
                "annotations": "duration,distance",
            }
            if hints is not None:
                params["hints"] = ";".join(hints[k] for k in idx)
            answer = _get("table", [coords[k] for k in idx], **params)
            block_tt = np.array(answer["durations"], dtype=float)  # null -> nan
            block_d = np.array(answer["distances"], dtype=float)
            if block_tt.shape != (len(src), len(dst)) or block_d.shape != block_tt.shape:
                raise RuntimeError(f"OSRM table block ({i0},{j0}) has shape {block_tt.shape}, "
                                   f"expected {(len(src), len(dst))}")
            tt[np.ix_(src, dst)] = block_tt
            dist[np.ix_(src, dst)] = block_d
    LOG.debug(f"OSRM table {n}x{n} filled, {int(np.isnan(tt).sum())} unroutable pairs")
    return tt, dist


def route_shape(coords):
    """Road geometry of the fastest route through the given (lon, lat) points, as a list of (lon, lat)."""
    answer = _get("route", coords, overview="full", geometries="geojson")
    return answer["routes"][0]["geometry"]["coordinates"]
