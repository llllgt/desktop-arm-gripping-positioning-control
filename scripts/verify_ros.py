"""Run two independent ROS processes in the activated official ROS environment."""
import os
import argparse
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'results/ros2')
output=parser.parse_args().output.resolve()
output.mkdir(parents=True,exist_ok=True)
env=os.environ.copy()
env['ROS_DOMAIN_ID']='47'
env['PYTHONUNBUFFERED']='1'
with (output/'controller.log').open('w',encoding='utf-8') as log:
    server=subprocess.Popen([sys.executable,'-m','desktop_arm.ros_nodes','server','--output',str(output)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        time.sleep(2)
        checks=subprocess.run([sys.executable,str(ROOT/'scripts/check_ros_actions.py'),'--output',str(output)],cwd=ROOT,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60)
        (output/'action_checks.log').write_text((checks.stdout+'\n'+checks.stderr).rstrip()+'\n',encoding='utf-8')
        if checks.returncode:
            raise RuntimeError('ROS action validation failed; see action_checks.log')
        client=subprocess.run([sys.executable,'-m','desktop_arm.ros_nodes','task'],cwd=ROOT,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=180)
        (output/'task.log').write_text(client.stdout+'\n'+client.stderr,encoding='utf-8')
        print(client.stdout,flush=True)
        print(client.stderr,flush=True)
        if client.returncode:
            raise RuntimeError(f'ROS client failed: {client.returncode}')
        server.wait(timeout=15)
        if server.returncode:
            raise RuntimeError(f'ROS server failed: {server.returncode}')
    finally:
        if server.poll() is None:
            server.terminate();server.wait(timeout=10)
