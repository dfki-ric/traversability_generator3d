#!/bin/bash

DEPS="doxygen libeigen3-dev libcgal-dev libpcl-dev libboost-filesystem-dev libboost-serialization-dev libboost-system-dev libopenscenegraph-dev libyaml-cpp-dev"

if dpkg --verify $DEPS; then
  echo "All OS dependencies are installed already. To update them run:"
  echo "# apt-get update && apt-get upgrade"
else
  sudo apt-get update
  sudo apt-get install -y $DEPS
fi
