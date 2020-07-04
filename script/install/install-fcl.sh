#/bin/sh -

git clone https://github.com/eigenteam/eigen-git-mirror
cd eigen-git-mirror
mkdir build
cd build
cmake ..
make -j"$(nproc)" && make install
cd /

git clone https://github.com/flexible-collision-library/fcl
cd fcl
git checkout 97455a4            # v0.6.1
cmake . -DFCL_WITH_OCTOMAP=off
make -j"$(nproc)" && make install && ldconfig
