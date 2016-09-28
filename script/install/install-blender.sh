#!/bin/sh -e

cd /usr/local
wget -q -O - http://mirror.cs.umn.edu/blender.org/release/Blender2.77/blender-2.77a-linux-glibc211-x86_64.tar.bz2 | tar xjf -
ln -s /usr/local/blender-2.77a-linux-glibc211-x86_64/blender /usr/local/bin
