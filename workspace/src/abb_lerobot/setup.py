import os

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop
from glob import glob

package_name = 'abb_lerobot'
submodules = 'abb_lerobot/submodules'


# colcon/ament_python may invoke:
#   setup.py develop --uninstall --editable --build-directory <dir>
# New setuptools versions removed/changed some of these options.
# Provide a tolerant develop command that accepts and ignores them.
class develop(_develop):
    user_options = _develop.user_options + [
        ('uninstall', None, 'colcon legacy option (ignored)'),
        ('editable', None, 'colcon option (ignored, develop is already editable)'),
        ('build-directory=', None, 'colcon option (ignored)'),
    ]

    boolean_options = getattr(_develop, 'boolean_options', []) + ['uninstall', 'editable']

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None

    def finalize_options(self):
        super().finalize_options()

    def run(self):
        # We intentionally ignore --uninstall/--build-directory. The normal develop run will
        # refresh the egg-link in the install space.
        return super().run()


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='artemkondratev5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = abb_lerobot.teleop_node:main',
            'lerobot_bridge = abb_lerobot.lerobot_bridge:main',
            'dataset_recorder = abb_lerobot.dataset_recorder:main',
            'gamepad_teleop = abb_lerobot.gamepad_teleop:main',
        ],
    },
    cmdclass={'develop': develop},
)
