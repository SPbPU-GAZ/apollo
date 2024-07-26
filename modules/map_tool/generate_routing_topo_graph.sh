#!/usr/bin/env bash

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${DIR}/apollo_base.sh"

# generate routing_map.bin in map directory.
${APOLLO_BIN_PREFIX}/modules/routing/topo_creator \
  --flagfile=modules/routing/conf/routing.conf \
  $@
