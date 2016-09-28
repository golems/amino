#/bin/sh -

wget -q -O - https://github.com/flexible-collision-library/fcl/archive/0.4.0.tar.gz | tar xzf -
cd fcl-0.4.0
cmake .
make -j 4 && sudo make install
