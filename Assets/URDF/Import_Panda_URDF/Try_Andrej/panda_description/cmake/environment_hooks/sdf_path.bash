#!/usr/bin/env bash

PKG_SHARE_DIR="$(cd "$(dirname "$(dirname "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
ament_prepend_unique_value SDF_PATH "${PKG_SHARE_DIR}"
