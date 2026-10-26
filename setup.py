from setuptools import setup
import os
from glob import glob

package_name = 'gn10_shot_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gn10',
    maintainer_email='gn10@todo.todo',
    description='TUI-based Shooting Point Editor & Trial Synchronous Recorder for GN10',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shooting_editor = gn10_shot_manager.shooting_editor:main',
            'trial_recorder = gn10_shot_manager.trial_recorder:main',
            'trial_viewer = gn10_shot_manager.trial_viewer:main',
        ],
    },
)

