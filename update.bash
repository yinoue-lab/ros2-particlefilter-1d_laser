#!/bin/bash

DIR_NAME="$(dirname $0)"

git submodule update --init --recursive

# Extentions
rm -rf $DIR_NAME/build $DIR_NAME/install $DIR_NAME/log
vcs import --recursive < $DIR_NAME/ext.repos
vcs pull $DIR_NAME/ext

ansible-playbook --ask-become-pass $DIR_NAME/ext/blackbox/ansible/dev.yml 

sudo apt update && rosdep update
rosdep install -yi --rosdistro humble --from-paths $DIR_NAME