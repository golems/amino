#!/bin/sh -e

git clone https://github.com/sbpl/sbpl.git
cd sbpl
git checkout 4d65484            # v1.3.1
mkdir build
cd build
cmake ..
make -j"$(nproc)" && make install
cd /
