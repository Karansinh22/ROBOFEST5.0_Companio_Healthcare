#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
docker stop "$CONTAINER_NAME" && echo "Container '$CONTAINER_NAME' stopped."
