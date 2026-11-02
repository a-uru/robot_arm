from setuptools import find_packages, setup
setup(name="so101_servo", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/so101_servo"]),
                  ("share/so101_servo", ["package.xml"])],
      install_requires=["setuptools", "numpy"], tests_require=["pytest"], zip_safe=True,
      maintainer="akira", maintainer_email="akira@example.com", license="Apache-2.0",
      description="SO-101 position-only target tracker for MoveIt Servo",
      entry_points={"console_scripts": ["point_tracker = so101_servo.point_tracker:main",
                                        "interactive_target = so101_servo.interactive_target:main"]})
