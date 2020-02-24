## Build
Build from source with cmake
```
cd Stereo3D
mkdir build
cmake ..
make
```

Build debian package
```
cpack
```

## Install
Option 1 (dpkg): Install with .deb package
```
dpkg -i stereo3d-X.X.X-Linux.deb
```
Option 2 (cmake): Install from cmake
```
sudo make install
```

## Uninstall
Option 1 (dpkg): Uninstall with dpkg
```
dpkg -r stereo3d
```
Option 2 (cmake): Uninstall using cmake
```
make uninstall
```