"""Set up the autonomous mobile robot package and Robot Operating System 2 data."""
from setuptools import setup
from glob import glob
import os

package_name='amr_lab'
setup(name=package_name,version='0.1.0',packages=['amr_lab','amr_lab.core','amr_lab.core.allocators','amr_lab.core.conflicts','amr_lab.nodes'],data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',glob('launch/*.py')),('share/'+package_name+'/config',glob('config/*.yaml')),('share/'+package_name+'/rviz',glob('rviz/*'))],install_requires=['setuptools','numpy','scipy'],zip_safe=True,maintainer='AMR Lab',maintainer_email='maintainer@example.com',description='Deterministic multi-robot warehouse simulator',license='Apache-2.0',entry_points={'console_scripts':['sim_node=amr_lab.nodes.sim_node:main','robot_node=amr_lab.nodes.robot_node:main','task_server_node=amr_lab.nodes.task_server_node:main','metrics_node=amr_lab.nodes.metrics_node:main']})
