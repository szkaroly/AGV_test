from setuptools import setup

setup(name='ginop_control',
      version='0.2',
      description='Package that contains a set of algorithms and classes to move AGVs in VREP,
      url='-',
      author='Dániel Rácz',
      author_email='racz.daniel.93@gmail.com',
      license='MIT',
      packages=['ginop_control', 'ginop_vrep', 'ginop_robots','vrep_norm'],
      install_requires=[
          'numpy',
          'pandas'
      ],
      include_package_data=True,
      zip_safe=False)
