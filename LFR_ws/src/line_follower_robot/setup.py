# Copyright 2025 Bishal Dutta (Bishalduttaoffcial@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages, setup
import os 
from glob import glob 
package_name = 'line_follower_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['LICENSE']), # Include the LICENSE file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
	    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
	    (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bishal Dutta',
    maintainer_email='Bishalduttaoffcial@gmail.com',
    description='A fast line follower robot simulation in ROS 2 and Gazebo.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = line_follower_robot.follower:main'
        ],
    },
)

