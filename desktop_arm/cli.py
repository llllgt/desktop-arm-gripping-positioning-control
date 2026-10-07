import argparse
import json
from pathlib import Path
from .paths import project_root


def main():
    parser = argparse.ArgumentParser(description='Desktop workpiece transfer tools')
    sub = parser.add_subparsers(dest='command',required=True)
    sim = sub.add_parser('simulate')
    sim.add_argument('--output',type=Path,default=project_root()/'results/native')
    sim.add_argument('--render',action='store_true')
    sim.add_argument('--order',type=int,choices=[3,5],default=5)
    sim.add_argument('--without-attachments',action='store_true')
    sim.add_argument('--no-gravity-compensation',action='store_true')
    args = parser.parse_args()
    if args.command == 'simulate':
        from .simulation import run
        print(json.dumps(run(args.output,order=args.order,render=args.render,fingers=not args.without_attachments,gravity_compensation=not args.no_gravity_compensation),indent=2))


if __name__ == '__main__':
    main()
