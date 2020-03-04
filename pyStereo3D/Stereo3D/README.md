# Stereo3D
Python package for generating 3D data from stereo cameras and images.

## Install
**Install python**  
[Windows]  
Download and install the latest windows release of python3 [here](https://www.python.org/downloads/release/python-368/)  
[Linux]
Download and install the latest linux release of python3 and python3-tk
```
sudo apt update
sudo apt install python3
sudo apt install python3-tk
```

**Install Pylon**  
Basler cameras require the software package Pylon to install the required drivers. Download and install it from [here](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/)  
***WARNING***: Make sure to download v1.5.1 as this is currently the only supported version for use with PyPylon. 

**Install PyPylon**  
PyPylon does not release wheels to pip so have to be installed manually.  
Find PyPylon GitHub Releases [here](https://github.com/basler/pypylon/releases)  
Download the wheel for your os and python version e.g. pypylon-1.5.1-cp36-cp36m-win_amd64.whl  

Install wheel:
```
python -m pip install pypylon-1.5.1-cp36-cp36m-OS.whl 
```

**Install Stereo3D**
```
python -m pip install stereo3d
```

## Examples
See [Samples](https://github.com/i3drobotics/Stereo3D/tree/master/pyStereo3D/SampleScripts) for example scripts of using this package.

