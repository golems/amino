#/bin/sh -

cd /
git clone https://github.com/OctoMap/octomap.git
cd octomap
git checkout 0b7eb0e            # 1.9.1
cmake .
make -j"$(nproc)"
make install
ldconfig
