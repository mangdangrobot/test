from setuptools import setup
from pathlib import Path
import os

package_name = 'mini_pupper_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [str(path) for path in Path('launch').glob('*.launch.py')]),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='0nhc, Herman Ye',
    author_email='thefoxfoxfox@outlook.com, hermanye233@icloud.com',
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='Example demo files for the Mini Pupper robot.',
    license='Apache-2.0',
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Framework :: Robot Operating System :: 2 :: Humble',
        'Topic :: Software Development',
    ],
    keywords=['ROS 2', 'mini_pupper', 'examples'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracking = '
            'mini_pupper_examples.object_tracking.object_tracking:main',
            'human_head_pose_estimation = '
            'mini_pupper_examples.head_pose_synchronization.'
            'human_head_pose_estimation:main',
            'pupper_head_pose_sync = '
            'mini_pupper_examples.head_pose_synchronization.'
            'pupper_head_pose_sync:main',
        ],
    },
)
