"""Submit improved programs from submissions/ to the hackathon site and report the result.

    python3 -m solver.submit            # each protocol whose local best beats what we submitted
    python3 -m solver.submit --all      # every protocol
    python3 -m solver.submit replicate  # just these

The team token is read from $C5R_TOKEN or the git-ignored file .team-token.
The site scores at most 2 submissions per team at a time and 30 per 10
minutes, so this runs two at a time.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SITE = os.environ.get("C5R_SITE", "https://stanford-bb1494.us-west-1.aws.c5r.net")
BEST = ROOT / "submissions"
SUBMITTED = BEST / "submitted.json"


def token():
    value = os.environ.get("C5R_TOKEN") or ((ROOT / ".team-token").read_text().strip() if (ROOT / ".team-token").exists() else "")
    if not value:
        sys.exit("no team token: set C5R_TOKEN or put it in .team-token")
    return value


def call(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(SITE + path, data=data, headers={"Content-Type": "application/json"}, method="POST" if body else "GET")
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as error:
        detail = error.read().decode(errors="replace")
        try:
            detail = json.loads(detail).get("error", detail)
        except ValueError:
            pass
        raise RuntimeError(f"{error.code}: {detail}") from None


def submit_one(name, secret):
    program = (BEST / f"{name}.txt").read_text()
    for _ in range(60):
        try:
            result = call("/api/hamilton", {"token": secret, "protocol": name, "program": program})
            break
        except RuntimeError as error:
            if "earlier submissions" not in str(error):
                return name, {"error": str(error)}
            time.sleep(5)
    else:
        return name, {"error": "the site kept saying to wait"}
    while result.get("status") != "done":
        time.sleep(2)
        result = call(f"/api/runs/{result['id']}")
    return name, result


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python3 -m solver.submit")
    parser.add_argument("protocols", nargs="*")
    parser.add_argument("--all", action="store_true", help="submit every protocol, improved or not")
    args = parser.parse_args(argv)
    secret = token()
    print("team:", call("/api/me", {"token": secret}).get("team"))
    local = json.loads((BEST / "scores.json").read_text())
    sent = json.loads(SUBMITTED.read_text()) if SUBMITTED.exists() else {}
    names = args.protocols or [n for n in sorted(local) if args.all or local[n] < sent.get(n, float("inf")) - 1e-3]
    if not names:
        print("nothing new to submit")
        return 0
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda n: submit_one(n, secret), names))
    for name, result in results:
        if "error" in result:
            print(f"{name:18} ERROR {result['error']}")
            continue
        if result.get("valid"):
            sent[name] = min(sent.get(name, float("inf")), result["score"])
            print(f"{name:18} {result['score']:8.2f} s  rank {result.get('rank')}  {SITE}{result.get('url', '')}")
        else:
            print(f"{name:18} INVALID on the site: " + "; ".join(result.get("failures", [])[:5]))
    SUBMITTED.write_text(json.dumps(dict(sorted(sent.items())), indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
