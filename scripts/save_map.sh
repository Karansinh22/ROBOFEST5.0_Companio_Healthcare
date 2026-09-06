#!/bin/bash
# Save the map currently published by GMapping into companio_description/maps/.
# Rooms recorded during mapping are already stored in <map_name>.locations.yaml.
#   ./scripts/save_map.sh [map_name]      (default: hospital_map)
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
MAP_NAME="${1:-hospital_map}"
run_in_container "rosrun map_server map_saver -f \$(rospack find companio_description)/maps/$MAP_NAME" \
  && echo "Saved: src/companio_description/maps/$MAP_NAME.{pgm,yaml}  (rooms: $MAP_NAME.locations.yaml)"
