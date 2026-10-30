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
    maintainer='Gento Aiba',
    maintainer_email='contact0@gn10.org',
    description='TUI-based Shooting Point Editor & Trial Synchronous Recorder for NHK Kousen Robocon 2026',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shooting_editor = gn10_shot_manager.shooting_editor:main',
            'shooting_gui = gn10_shot_manager.shooting_gui:main',
            'trial_recorder = gn10_shot_manager.trial_recorder:main',
            'trial_viewer = gn10_shot_manager.trial_viewer:main',
        ],
    },
)

