#!/bin/bash

set -e

if [ ekf_generator.py -nt replay_src/ekf.h ]; then
    ./ekf_generator.py
    mv ekf.h replay_src
    mv ekf.c replay_src
fi

gcc replay_src/main.c replay_src/ekf.c -lm -g -o replay

./replay $1 $2 plot_data.json
./plotter.py plot_data.json
