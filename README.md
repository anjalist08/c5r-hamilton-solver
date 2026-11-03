# c5r-hamilton-solver

A solver for the **Hamilton Speedrun** at the C5R × Stanford hackathon: it turns a
pipetting protocol into a firmware program for a Hamilton STARlet, as fast as the
digital twin allows.

The kit itself (grader, twin, decks, protocols) belongs to the organizers and is
not in this repo. Download it from the event site and unzip it into `kit/`:

```sh
git clone https://github.com/<you>/c5r-hamilton-solver && cd c5r-hamilton-solver
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
