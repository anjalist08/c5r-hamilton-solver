"""CLI.

    python3 -m solver solve PROTOCOL > program.txt   # print our program
    python3 -m solver bench [PROTOCOL ...]           # solve and score every protocol
"""

from __future__ import annotations

import argparse
import sys

from .bench import bench, solve


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python3 -m solver")
    commands = parser.add_subparsers(dest="command", required=True)
    one = commands.add_parser("solve", help="print the program for a protocol")
    one.add_argument("protocol")
    one.add_argument("--solver", choices=["ours", "starter"], default="ours")
    many = commands.add_parser("bench", help="solve and score protocols (default: all in the kit)")
    many.add_argument("protocols", nargs="*")
    many.add_argument("--solver", choices=["ours", "starter"], default="ours")
    many.add_argument("--submit", action="store_true", help="submit each new best to the site (python3 -m solver.submit)")
    args = parser.parse_args(argv)
    if args.command == "bench":
        return bench(args)
    print(solve(args.protocol, args.solver), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
