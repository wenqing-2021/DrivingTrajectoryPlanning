#!/bin/bash

cd build/install/src
# export BOKEH_ALLOW_WS_ORIGIN=127.0.0.1:5006
bokeh serve --show main.py --args "$@"