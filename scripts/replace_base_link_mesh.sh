#!/usr/bin/env bash
set -euo pipefail

UPSTREAM_REPO="${1:-../ws_excavator}"
SOURCE="${UPSTREAM_REPO}/src/excavator_description/meshes/base_link.STL"
DEST="src/excavator_description/meshes/base_link.STL"

if [[ ! -f "${SOURCE}" ]]; then
  echo "Cannot find upstream base mesh: ${SOURCE}" >&2
  echo "Pass the local ws_excavator checkout path as the first argument." >&2
  exit 1
fi

cp -- "${SOURCE}" "${DEST}"
echo "Restored the original upstream base_link.STL at ${DEST}"
