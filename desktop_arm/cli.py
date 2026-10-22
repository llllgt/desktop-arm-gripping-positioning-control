import argparse
import json
from pathlib import Path
from .paths import project_root


def main():
    parser = argparse.ArgumentParser(description='Four-DOF arm gripping, positioning and motion control')
    sub = parser.add_subparsers(dest='command',required=True)
    sim = sub.add_parser('simulate')
    sim.add_argument('--output',type=Path,default=project_root()/'results/native')
    sim.add_argument('--render',action='store_true')
    sim.add_argument('--order',type=int,choices=[3,5],default=5)
    sim.add_argument('--without-attachments',action='store_true')
    sim.add_argument('--no-gravity-compensation',action='store_true')
    sim.add_argument('--config',type=Path)
    sim.add_argument('--preflight',action='store_true',help='Reject sampled fixture intersections before simulation')
    assess = sub.add_parser('assess',help='Write preflight and fixed-height workspace grid')
    assess.add_argument('--config',type=Path)
    assess.add_argument('--order',type=int,choices=[3,5],default=5)
    assess.add_argument('--output',type=Path,default=project_root()/'results/assessment')
    planner = sub.add_parser('plan',help='Search a collision-checked transfer route and save a resolved task')
    planner.add_argument('--config',type=Path)
    planner.add_argument('--settings',type=Path)
    planner.add_argument('--output',type=Path,required=True)
    planner.add_argument('--execute',action='store_true')
    planner.add_argument('--render',action='store_true')
    args = parser.parse_args()
    from .task import load_task
    try:
        task = load_task(args.config)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if args.command == 'plan':
        from .planning import run
        if args.render and not args.execute:
            parser.error('--render requires --execute')
        try:
            settings = None if args.settings is None else json.loads(args.settings.read_text(encoding='utf-8'))
            report = run(args.output,task,settings=settings,execute=args.execute,render=args.render)
        except (ValueError, OSError) as error:
            parser.error(str(error))
        print(json.dumps(report,indent=2))
        if not report['success'] or ('execution' in report and
                (not report['execution']['success'] or not report['observed_opening_within_envelope'])):
            parser.exit(2)
    elif args.command == 'simulate':
        from .simulation import run
        if args.preflight:
            from .assessment import preflight
            if args.without_attachments:
                parser.error('--preflight uses the CAD attachments; remove --without-attachments')
            report = preflight(task,order=args.order)
            if not report['feasible']:
                parser.error(report['rejection'])
        print(json.dumps(run(args.output,task=task,order=args.order,render=args.render,fingers=not args.without_attachments,gravity_compensation=not args.no_gravity_compensation),indent=2))
    elif args.command == 'assess':
        from .assessment import run
        report = run(args.output,task=task,order=args.order)
        print(json.dumps(report,indent=2))
        if not report['feasible']:
            parser.exit(2)


if __name__ == '__main__':
    main()
