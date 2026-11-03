"""Turn a protocol into jobs, schedule them in rounds of up to eight channels.

A job is what one tip does between pickup and eject:

- aspirate once from a source well (enough for every dispense), then
- dispense into one or more targets: jets (tip stays dry) for all but the
  last, and the last may touch the liquid and mix (rule 5), then
- put any leftover back into the source (dm4), and eject.

A mix that is not right after a transfer into its well becomes a job of its
own: a fresh tip that only mixes.

Each round picks up to eight ready jobs, gives each a channel (its source
row when it can, so a plate column is one command), and runs them phase by
phase: one pickup, one aspirate, one dispense per dispense index, one empty,
one eject. The Robot splits a phase into several commands when the channels
cannot reach their wells at once.
"""

from __future__ import annotations

import json
import math
import os

from speedrun import protocol
from speedrun.robot import BELOW_SURFACE_MM, FLOOR_MM, Robot, split

from .rounds import plan_rounds

ROWS = "ABCDEFGHIJKLMNOP"


class Job:
    def __init__(self, source, first):
        self.source = source  # well ref, or None for a mix-only job
        self.drops = []  # (target, ul, step index)
        self.mix = None  # (ul, cycles): after the last drop, or the whole job for a mix-only job
        self.mix_well = None
        self.steps = {first}
        self.order = first

    @property
    def well(self):
        return self.source or self.mix_well

    def row(self):
        name = self.well.split(":")[1]
        return ROWS.index(name[0]) if name[0] in ROWS else 0


def solve(loaded, deck, options=None):
    options = {**json.loads(os.environ.get("SOLVER_OPTS", "{}")), **(options or {}), "deck": deck}
    steps = loaded["steps"]
    jobs, moves = build_jobs(steps, deck, options)
    return schedule(steps, jobs, moves, deck, options)


# --- jobs -------------------------------------------------------------------


def build_jobs(steps, deck, options):
    before = protocol.predecessors(steps)
    after = descendants(steps, before)
    volume = {ref: well["volume_microliters"] for ref, well in protocol.wells(deck).items()}
    wells = protocol.wells(deck)
    max_ul = max_tip_ul(deck)

    attached = {}  # transfer index -> mix index
    last_touch = {}
    for i, step in enumerate(steps):
        if step["op"] == "mix":
            prev = last_touch.get(step["well"])
            if prev is not None and steps[prev]["op"] == "transfer" and steps[prev]["to"] == step["well"]:
                if prev not in attached:
                    attached[prev] = i
        for ref in protocol.touches(step)[0]:
            last_touch[ref] = i

    jobs, moves, open_job = [], [], {}
    owner = {}
    for i, step in enumerate(steps):
        if step["op"] == "move":
            moves.append(i)
            continue
        if step["op"] == "mix":
            if any(attached.get(t) == i for t in attached):
                continue
            job = Job(None, i)
            job.mix, job.mix_well = (step["ul"], step["cycles"]), step["well"]
            jobs.append(job)
            owner[i] = job
            for ref in protocol.touches(step)[0]:
                open_job.pop(ref, None)
            continue
        source, target = step["from"], step["to"]
        job = open_job.get(source)
        if job is not None and not can_join(job, i, before, after, owner, steps, volume, wells, max_ul, options):
            job = None
        if job is None:
            job = Job(source, i)
            job.start_volume = volume[source]
            jobs.append(job)
        job.drops.append((target, step["ul"], i))
        job.steps.add(i)
        owner[i] = job
        if i in attached:
            m = attached[i]
            job.mix, job.mix_well = (steps[m]["ul"], steps[m]["cycles"]), target
            job.steps.add(m)
            owner[m] = job
        # a well that changes closes the jobs that would read from it later
        open_job.pop(target, None)
        open_job[source] = None if i in attached else job
        volume[source] -= step["ul"]
        volume[target] += step["ul"]
    return jobs, moves


def can_join(job, i, before, after, owner, steps, volume, wells, max_ul, options):
    if job.mix is not None or not options.get("share", True):
        return False
    # no step outside the job may sit between a job member and step i
    for p in before[i]:
        if p not in job.steps and any(p in after[member] for member in job.steps):
            return False
    uls = [ul for _, ul, _ in job.drops] + [steps[i]["ul"]]
    drawn = aspirated(options["deck"], uls, max_ul)
    if drawn is None:
        return False
    height = protocol.liquid_height(wells[job.source], job.start_volume - drawn)
    return height - BELOW_SURFACE_MM >= FLOOR_MM + 0.05


def aspirated(deck, uls, max_ul):
    """Liquid a tip draws to jet each of uls, on the worst tip curve that holds it; None if none does."""
    worst = None
    for curve in deck["deck"]["correction_curves"]:
        if curve.get("conditions") or curve["points"][-1][0] < sum(uls) or curve["tip_capacity_microliters"] > max_ul * 1.25 + 1:
            continue
        drawn = to_liquid(curve["points"], sum(to_plunger(curve["points"], ul) for ul in uls))
        if drawn is not None:
            worst = drawn if worst is None else max(worst, drawn)
    return worst


def to_plunger(points, ul):
    for (a, b), (c, d) in zip(points, points[1:]):
        if a <= ul <= c:
            return b + (d - b) * (ul - a) / (c - a)
    return float("inf")


def to_liquid(points, plunger):
    for (a, b), (c, d) in zip(points, points[1:]):
        if b <= plunger <= d:
            return a + (c - a) * (plunger - b) / (d - b)
    return None


def max_tip_ul(deck):
    return max(rack["volume_ul"] for rack in deck["speedrun"]["tips"].values())


def descendants(steps, before):
    after = [set() for _ in steps]
    for i in reversed(range(len(steps))):
        for p in before[i]:
            after[p].add(i)
            after[p] |= after[i]
    return after


# --- schedule ---------------------------------------------------------------


class FastRobot(Robot):
    def __init__(self, deck, options):
        super().__init__(deck)
        self.options = options
        self.spot = {}

    def pick_row_matched(self, channels, need_ul):
        """One tip per channel, channel c from row c of one rack column if possible."""
        fitting = sorted(
            (name for name, rack in self.racks.items() if rack["volume_ul"] >= need_ul and self.free[name]),
            key=lambda name: -self.racks[name]["volume_ul"] if self.options.get("big_tips", True) else self.racks[name]["volume_ul"],
        )
        for name in fitting:
            free = set(self.free[name])
            for column in range(12):
                if all((column, c) in free for c in channels):
                    return self.take(name, [(c, column, c) for c in channels])
        for name in fitting:
            if len(self.free[name]) >= len(channels):
                spots = self.free[name][: len(channels)]
                return self.take(name, [(c, col, row) for c, (col, row) in zip(sorted(channels), spots)])
        raise ValueError(f"no tips left that hold {need_ul} uL")

    def take(self, name, picks):
        rack = self.racks[name]
        targets = []
        for channel, column, row in picks:
            self.free[name].remove((column, row))
            x, y = rack["a1_mm"][0] + column * rack["pitch_mm"], rack["a1_mm"][1] - row * rack["pitch_mm"]
            targets.append((channel, round(x * 10), round(y * 10)))
            self.tip[channel], self.held[channel], self.inside[channel] = name, 0, 0.0
            self.source.pop(channel, None)
            self.spot[channel] = (name, round(x * 10), round(y * 10))
        if rack["define"] not in self.defined:
            self.send("C0TT", rack["define"][4:])
            self.defined.add(rack["define"])
        top, bottom = round(rack["top_z_mm"] * 10), round((rack["top_z_mm"] - rack["stroke_mm"]) * 10)
        from speedrun.robot import lists

        for group in split(targets):
            xs, ys, mask = lists(group, 1, 2)
            self.send("C0TP", f"xp{xs}yp{ys}tm{mask}tt{rack['define'][6:8]}tp{top:04d}tz{bottom:04d}th2450td1")
        return name

    def drop(self, drops):
        """Dispense (channel, well, ul, mix, jet, empty) drops in as few commands as the arm allows."""
        by_mix = {}
        for drop in drops:
            by_mix.setdefault(drop[3] or None, []).append(drop)
        for mix, same in by_mix.items():
            targets = [(c, *self.xy(ref), ref, ul, jet, empty) for c, ref, ul, _, jet, empty in same]
            for group in split(targets):
                zs, volumes, modes = [], [], {}
                for channel, _, _, ref, ul, jet, empty in group:
                    if empty:
                        plunger, ul = self.held[channel], self.inside[channel]
                    else:
                        plunger = self.plunger(channel, ul)
                        if plunger > self.held[channel] + 1:
                            raise ValueError(f"channel {channel} holds too little to dispense {ul} uL")
                    self.held[channel] -= plunger
                    self.inside[channel] -= ul
                    self.volume[ref] += ul
                    surface = self.bottom(ref) + self.height(ref)
                    zs.append(self.jet_z(ref, surface) if jet else self.floor(ref, mix and mix[0]))
                    volumes.append(plunger)
                    modes[channel] = 4 if empty else 2
                fields = self.dispense_fields(group, zs, volumes, 2, mix)
                dm = " ".join(str(modes.get(ch, next(iter(modes.values())))) for ch in range(8))
                self.send("C0DS", fields.replace("dm2&", f"dm{dm}", 1))

    def jet_z(self, ref, surface):
        mode = self.options.get("jet", "top")
        if mode == "top":
            plate = protocol.plates(self.deck)[ref.split(":")[0]]
            top = self.origin[plate["name"]][2] + plate["size_mm"][2]
            return math.ceil((max(top - self.options.get("rim_mm", 3.0), surface + 1.5)) * 10)
        return math.ceil((surface + 1.5) * 10)

    def eject(self, channels=None):
        """Drop tips in the waste, or by default put each back into the spot it came from."""
        if not self.options.get("return", True):
            return super().eject(channels)
        channels = sorted(self.tip if channels is None else channels)
        from speedrun.robot import lists

        for name in {self.spot[c][0] for c in channels}:
            rack = self.racks[name]
            top, bottom = round(rack["top_z_mm"] * 10), round((rack["top_z_mm"] - rack["stroke_mm"]) * 10)
            targets = [(c, self.spot[c][1], self.spot[c][2]) for c in channels if self.spot[c][0] == name]
            for group in split(targets):
                xs, ys, mask = lists(group, 1, 2)
                self.send("C0TR", f"xp{xs}yp{ys}tp{top:04d}tz{bottom:04d}th2450te2450tm{mask}ti1")
        for channel in channels:
            for state in (self.tip, self.held, self.inside, self.source, self.spot):
                state.pop(channel, None)


def schedule(steps, jobs, moves, deck, options):
    before = protocol.predecessors(steps)
    robot = FastRobot(deck, options)
    done = set()
    pending = list(jobs)
    pending_moves = list(moves)
    while pending or pending_moves:
        ready_moves = [m for m in pending_moves if done.issuperset(before[m])]
        if ready_moves:
            m = ready_moves[0]
            robot.move(steps[m]["plate"], steps[m]["to"])
            done.add(m)
            pending_moves.remove(m)
            continue
        ready = [job for job in pending if all(p in done or p in job.steps for s in job.steps for p in before[s])]
        if not ready:
            raise ValueError("deadlock: no job is ready")
        ready.sort(key=lambda job: job.order)
        batch = greedy_rounds(ready)
        if len(ready) > 8 and options.get("anneal", True):
            batch = plan_rounds(ready, robot, batch)
        for chosen, channel_of in batch:
            run_round(robot, chosen, channel_of)
            for job in chosen:
                done |= job.steps
                pending.remove(job)
    return robot.program()


def greedy_rounds(ready):
    rounds, left = [], list(ready)
    while left:
        chosen, channel_of = pick_round(left)
        rounds.append((chosen, channel_of))
        left = [job for job in left if job not in chosen]
    return rounds


def pick_round(ready):
    chosen, channel_of, used = [], {}, set()
    for job in ready:
        row = job.row()
        if row < 8 and row not in used:
            chosen.append(job)
            channel_of[id(job)] = row
            used.add(row)
        if len(chosen) == 8:
            break
    for job in ready:
        if len(chosen) == 8:
            break
        if job in chosen:
            continue
        free = [c for c in range(8) if c not in used]
        channel_of[id(job)] = free[0]
        used.add(free[0])
        chosen.append(job)
    return chosen, channel_of


def run_round(robot, jobs, channel_of):
    channels = sorted(channel_of[id(job)] for job in jobs)
    need = max(job_ul(job) for job in jobs)
    robot.pick_row_matched(channels, need)
    by_channel = {channel_of[id(job)]: job for job in jobs}

    takers = [c for c, job in by_channel.items() if job.source]
    if takers:
        wells = [by_channel[c].source for c in takers]
        doses = [len(by_channel[c].drops) for c in takers]
        aspirate(robot, takers, wells, [by_channel[c] for c in takers])
    mixers = [c for c, job in by_channel.items() if not job.source]
    if mixers:
        for mix, group in group_by(mixers, lambda c: by_channel[c].mix).items():
            robot.mix([by_channel[c].mix_well for c in group], mix[0], mix[1], group)

    depth = max((len(by_channel[c].drops) for c in takers), default=0)
    for k in range(depth):
        active = [c for c in takers if k < len(by_channel[c].drops)]
        drops = []
        for c in active:
            job = by_channel[c]
            last = k == len(job.drops) - 1
            target, ul, _ = job.drops[k]
            # every drop is a jet except a last drop that mixes, which must touch the liquid;
            # an exact-draw tip empties itself (dm4) on its last drop
            drops.append((c, target, ul, last and job.mix, not (last and job.mix), last and job.exact))
        robot.drop(drops)
    leftovers = [c for c in takers if robot.inside.get(c, 0) > 0.05]
    if leftovers:
        robot.empty(leftovers)
    robot.eject()


def aspirate(robot, channels, wells, jobs):
    """Aspirate for every drop of each job in one command where the arm allows.

    A job draws exactly the liquid it delivers when its plunger allows all but
    the last drop and the last drop empties the tip (dm4); otherwise it draws
    the sum of each drop's plunger and empties the rest back afterwards.
    """
    targets = []
    for channel, ref, job in zip(channels, wells, jobs):
        uls = [ul for _, ul, _ in job.drops]
        exact = robot.plunger(channel, sum(uls))
        job.exact = robot.options.get("exact", True) and len(uls) > 1 and exact - sum(robot.plunger(channel, ul) for ul in uls[:-1]) >= 0
        plunger = exact if job.exact or len(uls) == 1 else sum(robot.plunger(channel, ul) for ul in uls)
        targets.append((channel, *robot.xy(ref), ref, plunger, robot.liquid(channel, plunger)))
    for group in split(targets):
        zs = []
        for channel, _, _, ref, plunger, liquid in group:
            robot.volume[ref] -= liquid
            depth = robot.height(ref) - BELOW_SURFACE_MM
            if depth < FLOOR_MM:
                raise ValueError(f"{ref} keeps too little liquid to aspirate {liquid:.1f} uL below its surface")
            zs.append(math.floor((robot.bottom(ref) + min(robot.options.get("aspirate_mm", 2.0), depth)) * 10))
            robot.held[channel] += plunger
            robot.inside[channel] += liquid
            robot.source[channel] = ref
        robot.send("C0AS", robot.aspirate_fields(group, zs, [p for *_, p, _ in group]))


def job_ul(job):
    if not job.source:
        return job.mix[0]
    return max(sum(ul for _, ul, _ in job.drops) * 1.1, job.mix[0] if job.mix else 0)


def group_by(items, key):
    groups = {}
    for item in items:
        groups.setdefault(key(item), []).append(item)
    return groups
