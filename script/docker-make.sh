#!/bin/sh -e

if [ -z "$JOBS" ]; then
    JOBS=8
fi

for DOCKERFILE in $@; do
    NAME="amino-${DOCKERFILE}-install"
    docker rm "$NAME" || true
    docker run --name="$NAME" "amino:$DOCKERFILE-dep" /bin/sh -c "cd /root/amino && autoreconf -i && ./configure --enable-demo-baxter && make -k -j $JOBS V=1 distcheck && make install" && \
        docker commit "$NAME" "amino:${DOCKERFILE}-install"
done
