#!/bin/sh

if [ -z "$JOBS" ]; then
    JOBS=8
fi

for DOCKERFILE in $@; do
      docker run "amino-$DOCKERFILE" /bin/sh -c "cd /root/amino && autoreconf -i && ./configure --enable-demo-baxter && make -k -j $JOBS check V=1 && make -k -j $JOBS distcheck";
done
