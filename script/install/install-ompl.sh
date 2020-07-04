#!/bin/sh -e
git clone https://github.com/ompl/ompl.git
cd ompl
git checkout 3a98d97            # v1.5.0
mkdir -p build/Release
cd build/Release
cmake ../..
sed -i "s%-L_link_dirs-NOTFOUND%%g" ompl.pc # OMPL bug in .pc file
make -j"$(nproc)" && make install
cd /
