#!/bin/sh -e

if [ -z "$JOBS" ]; then
    JOBS=8
fi

REGISTRY=ndantam

tag_push() {
    A="amino:${1}"
    R="${REGISTRY}/${A}"
    docker tag "$A" "$R" && docker push "$R"
}

for DOCKERFILE in $@; do
    tag_push "${DOCKERFILE}-dep"
    tag_push "${DOCKERFILE}-install"
done
