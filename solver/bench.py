"""Solve and score protocols with the kit's grader."""

from __future__ import annotations

import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from . import KIT, plan
from speedrun import grade, protocol, starter  # noqa: E402  (KIT is on sys.path)

OUT = Path(__file__).resolve().parent.parent / "out"


def solve(path, solver="ours"):
    loaded = protocol.load(path)
    if solver == "starter":
        return starter.solve(loaded, loaded["deck"])
    return plan.solve(loaded, loaded["deck"])


def bench_one(path, solver):
    start = time.time()
    loaded = protocol.load(path)
    try:
        program = solve(path, solver)
    except ValueError as error:
        return loaded["id"], {"valid": False, "failures": [f"solver: {error}"]}, time.time() - start
    OUT.mkdir(exist_ok=True)
    (OUT / f"{loaded['id']}.txt").write_text(program)
    report, trace = grade.score(loaded, program)
    (OUT / f"{loaded['id']}.trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in trace))
    (OUT / f"{loaded['id']}.report.json").write_text(json.dumps(report))
    return loaded["id"], report, time.time() - start


def bench(args):
    paths = args.protocols or sorted(str(p) for p in (KIT / "protocols").glob("*.json"))
    with ProcessPoolExecutor() as pool:
        results = list(pool.map(bench_one, paths, [args.solver] * len(paths)))
    total = 0.0
    for name, report, took in results:
        if report["valid"]:
            total += report["seconds"]
            print(f"{name:18} {report['seconds']:8.1f} s  {report['commands']:4} cmds  ({took:.0f} s to score)")
        else:
            print(f"{name:18}  INVALID  " + "\n    ".join(report["failures"][:8]))
    print(f"{'total':18} {total:8.1f} s")
    return 0 if all(report["valid"] for _, report, _ in results) else 1
