"""Check one Robot Operating System 2 (ROS 2) launch and its metrics."""
import os
import subprocess
from pathlib import Path

def test_launch_writes_metrics_and_exits(tmp_path):
    root=Path(__file__).resolve().parents[3]; env=os.environ.copy()
    env['AMR_RESULTS_DIR']=str(tmp_path/'run'); env['AMR_TRIALS_CSV']=str(tmp_path/'trials.csv')
    cmd=['ros2','launch','amr_lab','fleet.launch.py','headless:=true','task_count:=0','max_ticks:=5']
    result=subprocess.run(cmd,cwd=root,env=env,text=True,capture_output=True,timeout=30)
    assert result.returncode==0, result.stdout+'\n'+result.stderr
    assert 'tasks_completed' in (tmp_path/'trials.csv').read_text()
