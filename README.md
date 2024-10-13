 
    This file is part of Pose1.
 
    dq1 is free software: you can redistribute it and/or modify 
    it under the terms of the GNU General Public License as published 
    by the Free Software Foundation, either version 3 of the License, 
    or (at your option) any later version.
 
    dq1 is distributed in the hope that it will be useful, 
    but WITHOUT ANY WARRANTY; without even the implied warranty of 
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with dq1. If not, see <https://www.gnu.org/licenses/>.



    Filename README.md
    Author Jiawei ZHAO
	Version 1.0
	Date 2023-2024


# DualQuaternion-based robot control
This project is the implementation of dual quaternion based robot control platform. About dual quaternion, please refer to https://en.wikipedia.org/wiki/Dual_quaternion. This project is solely intended for personal practice and experimentation. It does not come with any warranty or guarantee of safety.

# How to install and use this library
This project has several dependencies. Ensure these dependencies are already installed in your environment; if not, install them first. Here are some steps you might need to follow to complete the installation:
## 1. Quadratic programming solver - [qpOASES](https://coin-or.github.io/qpOASES/doc/3.0/doxygen/index.html)
```bash
cd ~/Downloads
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build
cd build
sudo cmake .. -DBUILD_SHARED_LIBS=ON
sudo make install
```
The build variable `BUILD_SHARED_LIBS` is set to `ON` so the compiler will build qpOASES as a shared library. Dependencies of shared must be shared and dependencies of static must be static. If you perfer to install it as a static library, use the following instead and keep in mind that the dq1 library also must be installed as a static library when you install the dq1 library afterwards:
```bash
cd ~/Downloads
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build
cd build
sudo cmake .. -DBUILD_SHARED_LIBS=OFF
sudo make install
```
The default install path of qpOASES is `/usr/local/lib`, which is not in the dynamic linker's search path. this results in runtime error while the program tries to access the libqpOASES.so, complaining no such file or directory. So to fix this issue, we need to add the path `/usr/local/lib` to the dynamic linker's search path. Here is a permanent solution:
1. create a new conf file for storing the path `/usr/local/lib`
```bash
sudo nano /etc/ld.so.conf.d/local.conf
```

2. Add the following line to include `/usr/local/lib`
```bash
/usr/local/lib
```

3. Save the file by pressing crtl + s first, and then ctrl + x to exit the editor
4. Update the dynamic linker by running:
```bash
sudo ldconfig
```

## 2. Linear algebra template library - [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
```bash
cd ~/Downloads
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
sudo cmake .. 
sudo make install
```

## 3. [nlohmann/json](https://json.nlohmann.me/)
```bash
cd ~/Downloads
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
sudo cmake .. -DJSON_BuildTests=OFF
sudo make install
```
The test files are blocked from building for saving time.

After installing this three dependencies, dq1 can be installed and afterwards be used finally. Follow below steps to install dq1.

## 4. This library - dq1
```bash
cd ~/Downloads
git clone https://github.com/zhaojiawe392/dq1.git
cd dq1
mkdir build
cd build
sudo cmake ..
sudo make install
```
Here, the build variable `BUILD_SHARED_LIBS` is set to `ON` by default, but this is only possible if qpOASES was installed as a shared library as well in previous steps. If you want to keep installed static, use the following instead and keep in mind that the dependency qpOASES library also must be installed as a static library:
```bash
cd ~/Downloads
git clone https://github.com/zhaojiawe392/dq1.git
cd dq1
mkdir build
cd build
sudo cmake .. -DBUILD_SHARED_LIBS=OFF
sudo make install
```

## 5. Done
Try to run example1 through example2 and see what happens
```bash
./example1
```
```bash
./example2
```



