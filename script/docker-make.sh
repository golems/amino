#!/bin/sh -e

if [ -z "$JOBS" ]; then
    if [ -z `which nproc` ]; then
        JOBS=8
    else
        JOBS=$((2*`nproc`))
    fi
fi

for DOCKERFILE in $@; do
    NAME="amino-${DOCKERFILE}-install"
    # Remove prior image
    if [ `docker images $NAME | wc -l` -gt 1 ]; then
        docker rm "$NAME"
    fi
    # Run the tests and commit a new image
    docker run --security-opt seccomp=unconfined \
           --name="$NAME" "amino:$DOCKERFILE-dep" \
           /bin/sh -c "cd /root/amino && autoreconf -i && ./configure --enable-demo-baxter && make -k -j $JOBS V=1 distcheck && make install && ldconfig" \
        && docker commit "$NAME" "amino:${DOCKERFILE}-install"
done
