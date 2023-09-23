#!/bin/bash
path_to_pkg=`rospack find turtlebot3_datasets`/data
gdown -O $path_to_pkg/ir_labs.tar.gz https://drive.google.com/uc?id=1-QdzXIoh0ltdDNGuffLqzsPZLnLwSjAj && tar -xvf $path_to_pkg/ir_labs.tar.gz -C $path_to_pkg