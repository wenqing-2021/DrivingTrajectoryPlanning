#!/bin/bash
set -e

cd build/install/src
# export BOKEH_ALLOW_WS_ORIGIN=127.0.0.1:5006

RES_SAVE_PATH=${RES_SAVE_PATH:-"solve_results/latest"}

# 1) Solve first and save pb files.
python3 main.py --res_save_path "$RES_SAVE_PATH" "$@"

# 2) Visualize saved pb files with vis_main.
bokeh serve --show utils/visualization/vis_main.py --args --res_save_path "$RES_SAVE_PATH"