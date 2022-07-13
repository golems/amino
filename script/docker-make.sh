#!/bin/sh -e

if [ -z "$JOBS" ]; then
    if [ -z "`which nproc`" ]; then
        JOBS=8
    else
        JOBS=$((2*`nproc`))
    fi
fi

for DOCKERFILE in $@; do
    DEP="amino:${DOCKERFILE}-dep"
    IMAGE="amino:${DOCKERFILE}-install"
    NAME="amino-${DOCKERFILE}-install"
    # Remove prior image
    if [ `docker container ls --all --filter name="^${NAME}\$" | wc -l` -gt 1 ]
    then
        docker rm "$NAME"
    fi
    # Run the tests and commit a new image
    docker run --security-opt seccomp=unconfined \
           --name="$NAME" "$DEP" \
           /bin/sh -c "cd /root/amino && autoreconf -i && ./configure --enable-demo-baxter && make -k -j $JOBS V=1 distcheck && make install && ldconfig" \
        && docker commit "$NAME" "$IMAGE"
done
