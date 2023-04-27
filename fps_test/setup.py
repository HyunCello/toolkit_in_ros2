import glob
import os
from setuptools import setup

package_name = 'fps_test'

setup(
  name=package_name,
  version='0.0.0',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages',
    ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='yh',
  maintainer_email='jeremykim95@gmail.com',
  description='TODO: Package description',
  license='TODO: License declaration',
  tests_require=['pytest'],
  entry_points={
      'console_scripts': [
          'image_pubsub = fps_test.image_pubsub:main'
      ],
  },


)
