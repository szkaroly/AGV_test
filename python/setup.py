from setuptools import setup

setup(name='ginop_control',
      version='0.1',
      description='Package that contains algorithms and minor implementation for differential drive robot kinematics',
      url='-',
      author='Dániel Rácz',
      author_email='racz.daniel.93@gmail.com',
      license='MIT',
      packages=['ginop_control', 'ginop_vrep', 'vrep_norm'],
      install_requires=[
          'numpy',
          'pandas'
      ],
      include_package_data=True,
      zip_safe=False)
