#!/usr/bin/env python3
"""Bench the HTTP API against the same frozen OD pairs the in-process harness uses.

Two things at once:
  - correctness: all three algorithms must return identical durations, over HTTP
  - latency: p50/p95/p99 at concurrency 1 and at concurrency 16

If p50 at concurrency 1 is materially above the in-process number, the context
pool or the serialisation path is eating it.

usage: http_bench.py [--url http://127.0.0.1:8080] [--pairs data/build/od.json]
                     [--n 1000] [--concurrency 1,16]
"""
import argparse
import json
import statistics
import sys
import time
import http.client
import threading
import urllib.parse
from concurrent.futures import ThreadPoolExecutor

_local = threading.local()
_HOST = None


def conn():
    """One kept-alive connection per worker thread.

    urllib opens a fresh TCP connection per call, which on loopback costs more
    than the route it is measuring.
    """
    c = getattr(_local, "conn", None)
    if c is None:
        c = http.client.HTTPConnection(_HOST[0], _HOST[1], timeout=30)
        _local.conn = c
    return c


def pct(xs, p):
    if not xs:
        return 0.0
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round((len(xs) - 1) * p)))]


def fetch(path):
    c = conn()
    start = time.perf_counter()
    c.request("GET", path)
    body = json.loads(c.getresponse().read())
    return (time.perf_counter() - start) * 1000.0, body


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--pairs", default="data/build/od.json")
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--concurrency", default="1,16")
    ap.add_argument("--alg", default="bidir")
    args = ap.parse_args()

    global _HOST
    u = urllib.parse.urlparse(args.url)
    _HOST = (u.hostname, u.port or 80)

    pairs = json.load(open(args.pairs))["pairs"][: args.n]
    urls = {
        alg: [
            f"/v1/route?from={p['from'][0]},{p['from'][1]}"
            f"&to={p['to'][0]},{p['to'][1]}&alg={alg}"
            for p in pairs
        ]
        for alg in ("dijkstra", "astar", "bidir")
    }

    # --- correctness, over HTTP -------------------------------------------
    print(f"checking {len(pairs)} pairs x 3 algorithms over HTTP")
    ref, mismatches = None, 0
    settled = {}
    for alg in ("dijkstra", "astar", "bidir"):
        with ThreadPoolExecutor(max_workers=16) as ex:
            bodies = list(ex.map(lambda u: fetch(u)[1], urls[alg]))
        durations = [b["duration_s"] for b in bodies]
        settled[alg] = statistics.mean(b["debug"]["nodes_settled"] for b in bodies)
        if ref is None:
            ref = durations
        else:
            bad = [i for i, (a, b) in enumerate(zip(ref, durations)) if a != b]
            for i in bad[:5]:
                print(f"  MISMATCH pair {i} {alg}: {durations[i]} vs {ref[i]}")
            mismatches += len(bad)
    for alg, mean in settled.items():
        print(f"  {alg:<9} mean nodes settled {mean:>9.0f}")
    if mismatches:
        print(f"\n{mismatches} mismatches over HTTP - this is a gate", file=sys.stderr)
        return 1
    print("  all three agree on every pair\n")

    # --- latency -----------------------------------------------------------
    print(f"| Concurrency | p50 (ms) | p95 (ms) | p99 (ms) | max (ms) | req/s |")
    print("|---|---|---|---|---|---|")
    for c in [int(x) for x in args.concurrency.split(",")]:
        u = urls[args.alg]
        # Warm up; these samples are discarded.
        with ThreadPoolExecutor(max_workers=c) as ex:
            list(ex.map(lambda x: fetch(x)[0], u[:50]))
        wall = time.perf_counter()
        with ThreadPoolExecutor(max_workers=c) as ex:
            times = list(ex.map(lambda x: fetch(x)[0], u))
        wall = time.perf_counter() - wall
        print(
            f"| {c} | {pct(times, 0.50):.3f} | {pct(times, 0.95):.3f} | "
            f"{pct(times, 0.99):.3f} | {max(times):.3f} | {len(times) / wall:.0f} |"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
