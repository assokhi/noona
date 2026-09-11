#!/usr/bin/env python3
"""Watch the GitHub Actions run for a commit and print what failed.

usage: ci.py [sha-or-branch] [--wait]

Needs no token. Job logs do, so the workflow publishes its own log to the
`ci-logs` branch on failure and this reads that raw file instead.
"""
import json
import re
import subprocess
import sys
import time
import urllib.request

REPO = "assokhi/noona"
INTERESTING = re.compile(
    r"^(error|warning)(\[E\d+\])?[:\[]|-->|panicked|FAILED|assertion|MISMATCH|test result|"
    r"contraction moved|does not run from|is a gate"
)


def api(path):
    req = urllib.request.Request(f"https://api.github.com{path}")
    req.add_header("Accept", "application/vnd.github+json")
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


def fetch_log(sha):
    url = f"https://raw.githubusercontent.com/{REPO}/ci-logs/{sha[:7]}.log"
    for _ in range(12):
        try:
            with urllib.request.urlopen(url, timeout=60) as r:
                return url, r.read().decode(errors="replace")
        except Exception:  # noqa: BLE001
            time.sleep(10)
    return url, None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    wait = "--wait" in sys.argv
    ref = args[0] if args else "HEAD"
    sha = subprocess.check_output(["git", "rev-parse", ref], text=True).strip()

    run = None
    for _ in range(90):
        runs = api(f"/repos/{REPO}/actions/runs?head_sha={sha}&per_page=5")["workflow_runs"]
        if runs:
            run = runs[0]
            if not wait or run["status"] == "completed":
                break
            print(f"  {run['status']} ...", flush=True)
        else:
            print("  no run yet ...", flush=True)
        time.sleep(20)
    if not run:
        print(f"no run found for {sha[:7]}")
        return 2

    print(f"{sha[:7]} {run['head_branch']} -> {run['status']} / {run['conclusion']}")
    for job in api(f"/repos/{REPO}/actions/runs/{run['id']}/jobs")["jobs"]:
        for s in job["steps"]:
            if s["conclusion"] not in ("success", "skipped", None):
                print(f"  {s['conclusion'].upper()} at step: {s['name']}")
    if run["conclusion"] == "success":
        return 0

    url, text = fetch_log(sha)
    if text is None:
        print(f"no published log at {url}")
        return 1
    lines = text.splitlines()
    keep = []
    for i, ln in enumerate(lines):
        if INTERESTING.search(ln):
            keep.extend(lines[i : i + 6])
            keep.append("")
    print(f"--- {url} ({len(lines)} lines, showing {min(len(keep), 220)}) ---")
    print("\n".join(keep[:220]))
    return 1


if __name__ == "__main__":
    sys.exit(main())
