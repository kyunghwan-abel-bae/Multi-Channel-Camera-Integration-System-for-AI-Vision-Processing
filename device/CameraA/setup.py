from setuptools import setup, find_packages

setup(
    name='local_CameraA',
    version='0.1.0',
    packages=find_packages(include=['RebarAlignment', 'TuningView', 'ManualTuningView', 'AligningView', 'custom_steel_cfg', 'custom_steel_cfg.*', 'test_vid', 'test_vid.*', 'weights', 'weights.*'])
)