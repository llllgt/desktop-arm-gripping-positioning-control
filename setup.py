from pathlib import Path
from setuptools import setup

files=[('share/ament_index/resource_index/packages',['resource/desktop_arm']),('share/desktop_arm',['package.xml'])]
for folder in ('assets','config','cad','ros2'):
    groups={}
    for file in Path(folder).rglob('*'):
        if file.is_file() and '__pycache__' not in file.parts and 'generated' not in file.parts:
            destination=Path('share/desktop_arm')/file.parent
            if file.parent==Path('ros2/launch'):
                destination=Path('share/desktop_arm/launch')
            groups.setdefault(str(destination),[]).append(str(file))
    files.extend(groups.items())
distribution=setup(data_files=files)
# ROS Jazzy's colcon introspection serializes setup metadata with literal_eval.
# PEP 621 may leave a SpecifierSet here; normalize it to its public string form.
for owner in (distribution,distribution.metadata):
    value=getattr(owner,'python_requires',None)
    if value is not None:
        owner.python_requires=str(value)
