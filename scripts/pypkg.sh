#! /bin/bash

if [ "x$PROCVER" == "xgpu" ]; then
    python3 -m pip install --user $(cat /tmp/pydeps_gpu.txt)
    rm -rf /tmp/*
elif [ "x$PROCVER" == "xcpu" ]; then
    python3 -m pip install --user $(cat /tmp/pydeps_cpu.txt)
    rm -rf /tmp/*
else
    echo "$PROCVER"
    echo "ERROR, PLEASE ALLOCATE $PROCVER"
    exit 1
fi