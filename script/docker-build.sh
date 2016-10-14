#!/bin/sh -e

for DOCKERFILE in $@; do
      docker build -t "amino:$DOCKERFILE-dep" -f "script/docker/$DOCKERFILE" .;
done
