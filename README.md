# c5r-hamilton-solver

A solver for the **Hamilton Speedrun** at the C5R × Stanford hackathon: it turns a
pipetting protocol into a firmware program for a Hamilton STARlet, as fast as the
digital twin allows.

The kit itself (grader, twin, decks, protocols) belongs to the organizers and is
not in this repo. Download it from the event site and unzip it into `kit/`:

```sh
git clone https://github.com/anjalist08/c5r-hamilton-solver && cd c5r-hamilton-solver
unzip ~/Downloads/c5r-stanford-kit.zip && mv c5r-stanford-kit kit
xattr -dr com.apple.quarantine kit      # macOS: lets the twin run
```

Or point `C5R_KIT` at the kit's `hamilton` folder. Python 3.9+, standard library only.

## Use

```sh
python3 -m solver solve kit/hamilton/protocols/serial-dilution.json > program.txt
python3 -m solver bench                      # solve + score every protocol in the kit
python3 -m solver bench --solver starter     # the kit's starter, for comparison
```

`bench` writes each program, trace and report to `out/`. Open
`kit/hamilton/viewer/hamilton.html` with the deck, trace and report to replay a run.

## Scores (local grader, seconds, lower is better)

| Protocol | This solver | Kit starter | Kit reference |
| --- | --- | --- | --- |
| reagent-add | 63.4 | 216.9 | 1787.6 |
| serial-dilution | 175.2 | 227.3 | 3162.7 |
| cherry-pick | 244.5 | 393.0 | 1189.7 |
| replicate | 254.0 | 429.4 | 3533.1 |
| magnet-cleanup | 260.0 | 314.5 | 2853.8 |

## How it works

- **Jobs.** Transfers from one source well share a tip: one aspirate, then a jet
  dispense into each target (the tip never touches their liquid, rule 5). A mix
  right after a transfer rides on that transfer's dispense. Leftover liquid from
  the non-linear plunger curve goes back to the source.
- **Rounds.** Up to eight independent jobs run together, one per channel, with
  channel = plate row where possible so a column is one command.
- **Round planning.** Simulated annealing regroups the ready jobs to cut the
  number of X positions each command visits (each costs about 2.5 s in the twin).
- **Tips** are 1000 µL where available (higher flow limits) and go back into
  their own rack spot instead of the waste.
