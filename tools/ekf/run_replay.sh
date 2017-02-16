#!/bin/bash

set -e

if [ ekf_generator.py -nt replay_src/ekf.h ]; then
    ./ekf_generator.py > replay_src/ekf.h
fi

gcc replay_src/main.c -lm -o replay

./replay $1 $2 plot_data.json
./plotter.py plot_data.json
