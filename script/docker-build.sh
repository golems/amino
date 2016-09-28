#!/bin/sh

for DOCKERFILE in $@; do
      docker build -t "amino-$DOCKERFILE" -f "script/docker/$DOCKERFILE" .;
done
