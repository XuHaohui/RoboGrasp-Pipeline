#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"

source "$WORKSPACE_DIR/install/setup.bash"

echo "Piper workspace loaded with CycloneDDS"
