#!/bin/bash
# Build the persistent service client (in the container): /runs/lookout/bin/gzsvc
set -e
mkdir -p /runs/lookout/bin
g++ -O2 -std=c++17 /lookout/gzsvc.cc -o /runs/lookout/bin/gzsvc \
  $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)
echo "built /runs/lookout/bin/gzsvc"
