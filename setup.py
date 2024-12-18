from setuptools import setup

package_name = 'auto_car_core'

setup(
  name=package_name,
  version='0.0.0',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='jhk',
  maintainer_email='023433@gmail.com',
  description='TODO: Package description',
  license='TODO: License declaration',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      "autocar = auto_car_core.autocar:main",
      "imu_sensor = auto_car_core.imu_sensor:main",
    ],
  },
)
