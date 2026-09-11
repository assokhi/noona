#!/usr/bin/env python3
"""Wait for CI on a commit and print what mattered from its log.

usage: ci.py [sha-or-branch] [--full]

Uses no GitHub API at all: the workflow publishes every run's log to the
`ci-logs` branch and this polls the raw file, which has no meaningful rate
limit. Exits 0 on success, 1 on failure, 2 if no log turned up in 20 minutes.
"""
import re
import subprocess
import sys
import time
import urllib.request

REPO = "assokhi/noona"
INTERESTING = re.compile(
    r"^(error|warning)(\[E\d+\])?[:\[]|-->|panicked|FAILED|assertion|MISMATCH|test result|"
    r"contraction moved|does not run from|is a gate|pushed a rustfmt|Diff in"
)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    ref = args[0] if args else "HEAD"
    sha = subprocess.check_output(["git", "rev-parse", ref], text=True).strip()
    url = f"https://raw.githubusercontent.com/{REPO}/ci-logs/{sha[:7]}.log"

    text = None
    for i in range(80):
        try:
            with urllib.request.urlopen(url, timeout=60) as r:
                text = r.read().decode(errors="replace")
            break
        except Exception:  # noqa: BLE001
            if i % 4 == 0:
                print(f"  waiting for {sha[:7]} ...", flush=True)
            time.sleep(15)
    if text is None:
        print(f"no log after 20 minutes: {url}")
        return 2

    m = re.search(r"== outcome: (\w+) ==", text)
    outcome = m.group(1) if m else "unknown"
    lines = text.splitlines()
    print(f"{sha[:7]} -> {outcome}   ({len(lines)} log lines)  {url}")
    if "--full" in sys.argv:
        print(text)
    elif outcome != "success":
        keep = []
        for i, ln in enumerate(lines):
            if INTERESTING.search(ln):
                keep.extend(lines[i : i + 7])
                keep.append("")
        print("\n".join(keep[:260]))
    else:
        for ln in lines:
            if re.search(r"test result|pushed a rustfmt|fixture clip|release-assert:", ln):
                print("  " + ln.strip())
    return 0 if outcome == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
