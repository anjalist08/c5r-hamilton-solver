"""Solver for the C5R x Stanford Hamilton Speedrun.

The kit's `speedrun` package (protocol model, grader, Robot) is imported from
the kit folder: `kit/hamilton` in this repo, or the path in $C5R_KIT.
"""

import os
import sys
from pathlib import Path

KIT = Path(os.environ.get("C5R_KIT", Path(__file__).resolve().parent.parent / "kit" / "hamilton"))

if not (KIT / "speedrun").is_dir():
    sys.exit(f"kit not found at {KIT}: unzip the C5R kit into kit/ or set C5R_KIT to its hamilton folder")
sys.path.insert(0, str(KIT))
