#!/bin/sh -e

cd /usr/local

wget -O - 'http://prdownloads.sourceforge.net/sbcl/sbcl-1.3.9-x86-64-linux-binary.tar.bz2' | tar xjf -
cd sbcl*
sh install.sh
