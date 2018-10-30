#!/usr/bin/env bash
#title          :install-openvx:.sh
#description    :This script installs and configures openVX for the Odroid XU4.
#author         :Taha Azzaoui <tazzaoui@cs.uml.edu>
#date           :08.14.2018
#version        :1    
#usage          :bash install-openvx.sh
#=================================================================================

# Remove the old sample dir
rm -rf openvx_sample*

echo "Downloading sources..."
# Get the source (version 1.2)
wget https://www.khronos.org/registry/OpenVX/sample/openvx_sample_1.2.tar.bz2

# Unzip it
echo "Extracting sources...."
tar -xvf openvx_sample_1.2.tar.bz2
cd openvx_sample

# Replace conflicting achitecture info
grep -rl '-m64' ./ | xargs sed -i 's/-m64/ /g'
grep -rl '-m32' ./ | xargs sed -i 's/-m32/ /g'


echo "Building OpenVX 1.2 with tiling and openmp in debug mode..." 

# Build with tiling, openmp, and debugging 
python Build.py --os=Linux --tiling --openmp --conf=Debug --rebuild=True --c=gcc --cpp=g++

echo "Finished building."
echo "Updating default linker & compiler search paths in $HOME/.bashrc"

# Update default linker & compiler search paths
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/Linux/x64/Debug/bin" >> $HOME/.bashrc
echo "export PATH=$PATH:$(pwd)/install/Linux/x64/Debug/include" >> $HOME/.bashrc
echo "export LDFLAGS=\"-L$(pwd)/install/Linux/x64/Debug/bin\"" >> $HOME/.bashrc 
echo "export CFLAGS=\"-I$(pwd)/install/Linux/x64/Debug/include\"" >> $HOME/.bashrc 

# Update changes
source $HOME/.bashrc

echo "Compiling Example Kernels..."
cd ../examples
make
echo "Successfully built examples in $(pwd)/examples"
