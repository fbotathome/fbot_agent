from setuptools import setup
from glob import glob

package_name = 'fbot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.agents', package_name + '.tools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fbot',
    maintainer_email='fbot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fbot_agent_node = fbot_agent.fbot_agent_node:main'
        ],
    },
)
