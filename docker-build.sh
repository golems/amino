#!/bin/sh

for DOCKERFILE in $@; do
      docker build -t "amino-$DOCKERFILE" -f "share/docker/$DOCKERFILE" .;
done
