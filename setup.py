from glob import glob
from setuptools import setup, find_packages

setup(
    name='rm_phase3_odometry', version='0.1.0', packages=find_packages(exclude=['tests']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/rm_phase3_odometry']),
                ('share/rm_phase3_odometry', ['package.xml', 'README.md', 'VERIFICATION.md', 'EKF_CORRECTION.md']),
                ('share/rm_phase3_odometry/launch', glob('launch/*.launch.py')),
                ('share/rm_phase3_odometry/config', glob('config/*.yaml')),
                ('share/rm_phase3_odometry/driver_patch', glob('driver_patch/*'))],
    install_requires=['setuptools', 'numpy'], tests_require=['pytest'], zip_safe=True,
    maintainer='SCE 672 student', maintainer_email='student@example.com',
    description='Lecture 11 five-state odometry estimator and supervised AprilTag trials',
    license='MIT', entry_points={'console_scripts': [
        'nav_node = rm_phase3_odometry.nav_node:main',
        'camera_node = rm_phase3_odometry.camera_node:main',
        'video_diagnostics = rm_phase3_odometry.video_diagnostics:main',
        'trial_analysis = rm_phase3_odometry.trial_analysis:main']},
)
