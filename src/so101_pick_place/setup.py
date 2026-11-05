from glob import glob
from setuptools import setup
setup(name='so101_pick_place', version='0.1.0', packages=['so101_pick_place'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/so101_pick_place']),
                  ('share/so101_pick_place', ['package.xml']),
                  ('share/so101_pick_place/config', glob('config/*.yaml')),
                  ('share/so101_pick_place/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='akira',
      maintainer_email='akira@example.com', description='Enter-triggered SO-101 MoveIt pick and place',
      license='Apache-2.0', entry_points={'console_scripts': ['pick_place = so101_pick_place.node:main']})
