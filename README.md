# Multi-Channel Camera Integration System for AI-Vision Processing

[PORTFOLIO] This repository shows UI/UX and a part of the rebar processing based on AI-Vision system.

* main.py : A root of a backend implementation
* main.qml : A root of a frontend implementation
* device/DeviceManager.py : A class to manager all registered devices
* device/CameraA/AligningView.py : A class about real time rebar alignment based machine learning prediction

## Preview (it is a gif image, so it takes time to load)
<p align="center">
<img src="img/preview.gif">
</p>

## Environment
* Python 3.7.9
* Windows 10
* Qt 5.15.2 (MSVC 2019, 64bit)
* CUDA(11.0, 10.1), cuDNN(8.1, 7.6)

## Camera - Device function
* Camera A : Rebar Alignment
* Camera B : Rebar Twist
* Camera C : Shape Recognition
* Camera D : Rebar Load Status

## Steps to build
* The path location of this project should be English only, not the other characters(eg. Korean, ...)
* (optional, in Windows) python -m pip install --upgrade pip
* pip install wheel
* (if existed) Remove local-CameraA, local-CameraB, local-CameraC, local-MvImport in requirements.txt
* pip install -r requirements.txt
* set system variable
  * Name : QT_QPA_PLATFORM_PLUGIN_PATH
  * Value : {env}/Lib/site-packages/PySide6/plugins (ex. C:\Users\MDL\Desktop\project\venv\Lib\site-packages\PySide6\plugins)
* install cuda kit, cuCNN
* Download [required data](https://google.com), and put them as below
  * Camera A : {rebar2020 PATH}/device/CameraA/custom_steel_cfg
  * Camera A : {rebar2020 PATH}/device/CameraA/test_vid
  * Camera B : {rebar2020 PATH}/device/CameraB/avi, mp4 files
  * Camera C : {rebar2020 PATH}/device/CameraC/mp4 files
  * Camera C : {rebar2020 PATH}/device/CameraC/model_fine_final.h5
  * Camera C : {rebar2020 PATH}/device/CameraC/shape_test_vid
* pip install -e at {CameraA, CameraB, CameraC, MvImport}
* python main.py
