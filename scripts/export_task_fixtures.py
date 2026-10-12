"""Export the two support solids at the heights specified by a transfer task."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cadquery as cq
from cad.build import make_fixture
from desktop_arm.paths import project_root
from desktop_arm.task import load_task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    task = load_task(args.config)
    design = json.loads((project_root()/'cad/design.json').read_text(encoding='utf-8'))
    args.output.mkdir(parents=True, exist_ok=True)
    metrics = {}
    for label in ('pick', 'place'):
        height = (task[label][2]-task['workpiece_size'][2]/2)*1000
        part = make_fixture(design['fixture'], height)
        bounds = part.val().BoundingBox()
        assert part.val().isValid() and abs(bounds.zlen-height)<1e-8
        for suffix in ('step', 'stl'):
            cq.exporters.export(part, str(args.output/f'{label}_fixture.{suffix}'))
        metrics[label] = {'height_mm': height, 'volume_mm3': part.val().Volume(),
                          'valid_solid': part.val().isValid()}
    (args.output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
