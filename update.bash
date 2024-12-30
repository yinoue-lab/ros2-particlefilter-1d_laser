#!/bin/bash

DIR_NAME="$(dirname $0)"

# Extentions
rm -rf $DIR_NAME/build $DIR_NAME/install $DIR_NAME/log
vcs import --recursive < $DIR_NAME/ext.repos
vcs pull $DIR_NAME/ext
vcs pull $DIR_NAME/src

sudo apt update && rosdep update
rosdep install -yi --rosdistro humble --from-paths $DIR_NAME