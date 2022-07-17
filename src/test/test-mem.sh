#!/bin/sh

TEST_PROG=$(echo "$0" | sed -e 's/[_\\-]mem\(.sh\)\?$//')
TEST_PROG=$(basename "${TEST_PROG}")

if [ ! -e ./${TEST_PROG} ]; then
    >&2 echo "Could not find test program '${TEST_PROG}'. Aborting test."
    exit 1
fi

export AMINO_VALGRIND=1

exec ./libtool --mode=execute valgrind \
     --leak-check=full \
     --error-exitcode=1 \
     --show-leak-kinds=all \
     --show-reachable=no \
     ./${TEST_PROG}
