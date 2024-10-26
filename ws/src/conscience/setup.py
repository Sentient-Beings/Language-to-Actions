from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'conscience'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install Python scripts
        (os.path.join('lib', package_name), glob('conscience/*.py')),
    ],
    install_requires=[
        'setuptools',
        'redis',
        'streamlit',
    ],
    zip_safe=True,
    maintainer='ambi',
    maintainer_email='thisisdeahmed@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = conscience.agent:main',
            'image_server = conscience.image_server:main',
            'streamlit_app = conscience.streamlit_app:main',
        ],
    },
)
