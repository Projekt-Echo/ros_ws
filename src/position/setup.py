from setuptools import find_packages, setup

package_name = 'position'

setup(
	name=package_name,
	version='0.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
		 ['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='ssyximmlove',
	maintainer_email='ssyximmlove@todo.todo',
	description='TODO: Package description',
	license='MIT',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			"direct = position.direct:main",
			"fitting = position.fitting:main",
		],
	},
)
