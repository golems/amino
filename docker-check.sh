#!/bin/sh

for DOCKERFILE in $@; do
      docker run "amino-$DOCKERFILE" /bin/sh -c "cd /root/amino && autoreconf -i && ./configure --enable-demo-baxter && make -k -j $JOBS check && make -k -j $JOBS distcheck";
done
