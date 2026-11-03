"""Time single commands in the twin to learn its motion model.

    python3 -m solver.probe DECK PROGRAM [LINE FIELD VALUE ...]

Runs PROGRAM (optionally with field FIELD of line LINE replaced by VALUE)
and prints each command's ms. Example: does a higher aspirate save time?

    python3 -m solver.probe kit/hamilton/decks/serial-dilution.json out/serial-dilution.txt 3 zl 1950
"""

import re
import sys

from . import KIT  # noqa: F401  (puts the kit on sys.path)
from speedrun import twin


def patch(lines, line, field, value):
    wire = lines[line - 1]
    lines[line - 1] = re.sub(rf"(?<=[0-9&]){field}[0-9 ]+(&)?", lambda m: field + value + (m.group(1) or ""), wire, count=1) if field != "id" else wire
    return lines


def main(argv):
    deck, program, *edits = argv
    lines = [l for l in open(program).read().splitlines() if l and not l.startswith("#")]
    for i in range(0, len(edits), 3):
        patch(lines, int(edits[i]), edits[i + 1], edits[i + 2])
    trace = twin.run(deck, "\n".join(lines) + "\n")
    total = 0
    for row in trace:
        if "wire" in row:
            total += row["ms"]
            flag = " REFUSED" if row["refused"] else (" ERROR " + row["reply"] if row["error"] else "")
            print(f"{row['line']:4} {row['wire'][:4]} {row['ms']:6}{flag}")
    print("total ms", total)


if __name__ == "__main__":
    main(sys.argv[1:])
