"""Split a pool of independent ready jobs into rounds of up to eight channels.

The twin serves the channels of one command one X after another, so a
command costs roughly a fixed amount plus one "stop" per distinct X, and
channels at the same X whose rows do not fit their channel order force a
second command. Simulated annealing moves jobs between rounds to minimize
that estimate, trying a few channel assignments for each round.
"""

from __future__ import annotations

import math
import random

from speedrun.robot import split

COMMAND_S = 0.8 + 2.1  # overhead + first stop
STOP_S = 2.5  # each extra X in one command
ROUND_S = 10.0  # pickup + eject + their overhead


def plan_rounds(pool, robot, greedy, iterations=20000, seed=7):
    """Return [(jobs, {id(job): channel})] covering pool, starting from the greedy rounds."""
    xy = {}

    def where(ref):
        if ref not in xy:
            xy[ref] = robot.xy(ref)
        return xy[ref]

    start = [list(jobs) for jobs, _ in greedy]
    best = (sum(round_cost(r, where)[0] for r in start) + ROUND_S * len(start), start)
    for extra in (0, 1):
        if len(start) + extra > len(pool):
            continue
        rounds = anneal([list(r) for r in start] + [[] for _ in range(extra)], where, iterations, random.Random(seed))
        cost = sum(round_cost(r, where)[0] for r in rounds) + ROUND_S * len(rounds)
        if best is None or cost < best[0]:
            best = (cost, rounds)
    return [(jobs, round_cost(jobs, where)[1]) for jobs in best[1] if jobs]


def anneal(rounds, where, iterations, rng):
    count = len(rounds)
    costs = [round_cost(r, where)[0] for r in rounds]
    total = sum(costs)
    temp, end = 3.0, 0.02
    decay = (end / temp) ** (1 / max(1, iterations))
    for _ in range(iterations):
        a, b = rng.randrange(count), rng.randrange(count)
        if a == b or not rounds[a]:
            temp *= decay
            continue
        i = rng.randrange(len(rounds[a]))
        if len(rounds[b]) < 8 and rng.random() < 0.3:
            new_a = rounds[a][:i] + rounds[a][i + 1 :]
            new_b = rounds[b] + [rounds[a][i]]
        elif rounds[b]:
            j = rng.randrange(len(rounds[b]))
            new_a = rounds[a][:i] + [rounds[b][j]] + rounds[a][i + 1 :]
            new_b = rounds[b][:j] + [rounds[a][i]] + rounds[b][j + 1 :]
        else:
            temp *= decay
            continue
        ca, cb = round_cost(new_a, where)[0], round_cost(new_b, where)[0]
        delta = ca + cb - costs[a] - costs[b]
        if delta <= 0 or rng.random() < math.exp(-delta / temp):
            rounds[a], rounds[b], costs[a], costs[b] = new_a, new_b, ca, cb
            total += delta
        temp *= decay
    return rounds


def round_cost(jobs, where):
    """(estimated seconds, channel map) for the best of a few channel assignments."""
    if not jobs:
        return 0.0, {}
    best = None
    for assign in assignments(jobs, where):
        cost = phases_cost(jobs, assign, where)
        if best is None or cost < best[0]:
            best = (cost, assign)
    return best


def assignments(jobs, where):
    n = len(jobs)
    out = []
    rows = [job.row() for job in jobs]
    if len(set(rows)) == n and max(rows) < 8:
        out.append({id(job): row for job, row in zip(jobs, rows)})
    for key in (
        lambda job: -where(job.well)[1],
        lambda job: -where(job.drops[0][0] if job.drops else job.well)[1],
        lambda job: (where(job.well)[0], -where(job.well)[1]),
    ):
        ordered = sorted(jobs, key=key)
        out.append({id(job): channel for channel, job in enumerate(ordered)})
    return out


def phases_cost(jobs, assign, where):
    cost = 0.0
    takers = [job for job in jobs if job.source]
    phases = [[(assign[id(job)], *where(job.source)) for job in takers]]
    phases.append([(assign[id(job)], *where(job.well)) for job in jobs if not job.source])
    depth = max((len(job.drops) for job in takers), default=0)
    for k in range(depth):
        phases.append([(assign[id(job)], *where(job.drops[k][0])) for job in takers if k < len(job.drops)])
    if any(len(job.drops) > 1 for job in takers):
        phases.append([(assign[id(job)], *where(job.source)) for job in takers if len(job.drops) > 1])
    for targets in phases:
        for group in split([t for t in targets]):
            cost += COMMAND_S + STOP_S * (len({x for _, x, _ in group}) - 1)
    return cost
