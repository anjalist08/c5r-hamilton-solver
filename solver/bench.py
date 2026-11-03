"""Solve and score protocols with the kit's grader."""

from __future__ import annotations

import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from . import KIT, plan
from speedrun import grade, protocol, starter  # noqa: E402  (KIT is on sys.path)

OUT = Path(__file__).resolve().parent.parent / "out"
BEST = Path(__file__).resolve().parent.parent / "submissions"


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
    improved = keep_best(results)
    for name, report, took in results:
        if report["valid"]:
            total += report["seconds"]
            mark = "  NEW BEST -> submissions/" + name + ".txt" if name in improved else ""
            print(f"{name:18} {report['seconds']:8.1f} s  {report['commands']:4} cmds  ({took:.0f} s to score){mark}")
        else:
            print(f"{name:18}  INVALID  " + "\n    ".join(report["failures"][:8]))
    print(f"{'total':18} {total:8.1f} s")
    if getattr(args, "submit", False) and improved:
        from . import submit

        submit.main(improved)
    return 0 if all(report["valid"] for _, report, _ in results) else 1


def keep_best(results):
    """Copy each valid program that beats submissions/scores.json into submissions/; return their names."""
    BEST.mkdir(exist_ok=True)
    scores_path = BEST / "scores.json"
    scores = json.loads(scores_path.read_text()) if scores_path.exists() else {}
    improved = []
    for name, report, _ in results:
        if report["valid"] and report["seconds"] < scores.get(name, float("inf")) - 1e-6:
            (BEST / f"{name}.txt").write_text((OUT / f"{name}.txt").read_text())
            scores[name] = report["seconds"]
            improved.append(name)
    scores_path.write_text(json.dumps(dict(sorted(scores.items())), indent=2) + "\n")
    return improved
