from setuptools import find_packages, setup

package_name = 'qr_code_scanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qr_detector.launch.py', 'launch/qr_scanner.launch.py', 'launch/qr_scanner_full.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@example.com',
    description='Package de détection de QR codes pour le robot TRC X3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_detector = qr_code_scanner.qr_detector:main',
            'image_capturer = qr_code_scanner.image_capturer:main'
        ],
    },
)