#!/usr/bin/env bash
#title           :install-openvx:.sh
#description     :This script installs and configures openVX for the Odroid XU4.
#author		 :Taha Azzaoui <tazzaoui@cs.uml.edu>
#date            :08.14.2018
#version         :0    
#usage		 :bash install-openvx.sh
#=================================================================================


# remove the old sample dir
rm -rf openvx_sample

# Unzip it
tar -xvf openvx_sample_1.2.tar.bz2
cd openvx_sample

# Replace conflicting achitecture info
grep -rl '-m64' ./ | xargs sed -i 's/-m64/ /g'
grep -rl '-m32' ./ | xargs sed -i 's/-m32/ /g'

# Specify opencl paths
export VX_OPENCL_INCLUDE_PATH=/usr/share/mali/headers/CL
export VX_OPENCL_LIB_PATH=/usr/lib/arm-linux-gnueabihf/mali-egl/libOpenCL.so

# Build with tiling, openmp, and opencl support
python Build.py --os=Linux --tiling --openmp --opencl --rebuild=True --c=gcc --cpp=g++

# Configure linker search paths
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/Linux/x64/Release/bin" >> /home/odroid/.bashrc
sudo echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/Linux/x64/Release/bin" >> /root/.bashrc
sudo ln -s /home/odroid/Research2/openvx_sample/install/Linux/x64/Release/bin/*.so /usr/lib
sudo ln -s /home/odroid/Research2/openvx_sample/install/Linux/x64/Release/bin/*.a /usr/lib

# Copy the headers to System's default include path
sudo rm -r /usr/include/VX
sudo cp -r $(pwd)/install/Linux/x64/Release/include/VX /usr/include/VX

# To test that this worked..
#cd raw
#../install/Linux/x64/Release/bin/vx_test
