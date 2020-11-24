from setuptools import setup, find_packages

setup(name='narval_monitor',
      version='0.1',
      description='Interface for talking with the narval_monitor interface',
      author='Pierre Narvor',
      author_email='pierre.narvor@ensta-bretagne.fr',
      licence='confidential',
      packages=find_packages(include=['narval_monitor']),
      install_requires=[
        'numpy',
        'matplotlib',
        'requests'
      ],
      zip_safe=False)


