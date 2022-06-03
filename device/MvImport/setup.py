from setuptools import setup, find_packages

setup(
    name='local_MvImport',
    version='0.1.0',
    packages=find_packages(include=['RunnableStreaming', 'CameraParams_const', 'CameraParams_header', 'MvCameraControl_class', 'MvCameraControl_header', 'MvErrorDefine_const', 'PixelType_const', 'PixelType_header'])
)