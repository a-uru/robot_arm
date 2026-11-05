from glob import glob
from setuptools import setup

setup(name='so101_aruco', version='0.1.0', packages=['so101_aruco'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/so101_aruco']),
                  ('share/so101_aruco', ['package.xml']),
                  ('share/so101_aruco/launch', glob('launch/*.launch.py')),
                  ('share/so101_aruco/config', glob('config/*.yaml')),
                  ('share/so101_aruco/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='akira', maintainer_email='akira@example.com',
      description='ArUco detection and calibrated camera-relative marker poses.',
      license='Apache-2.0',
      entry_points={'console_scripts': ['detector = so101_aruco.node:main', 'preview = so101_aruco.preview:main']})
