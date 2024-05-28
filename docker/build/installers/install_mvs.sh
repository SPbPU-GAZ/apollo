#!/usr/bin/env bash

# Fail on first error.
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

TARGET_ARCH="$(uname -m)"

# Already installed
# apt_get_update_and_install \
    # unzip

VERSION="3.0.1"

PKG_MVS="MVS-${VERSION}"
mkdir "${PKG_MVS}" && cd "${PKG_MVS}"

PKG_ZIP="${PKG_MVS}.zip"
CHECKSUM="448468d863746a7721170353648a2314529aedbef0bb211d4929d1d2491b295d"
DOWNLOAD_LINK="https://cloud.spbpu.com/s/ciAJToTGQMtP8Xd/download"
download_if_not_cached "${PKG_ZIP}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

unzip "${PKG_ZIP}"

apt install -y ./${PKG_MVS}_${TARGET_ARCH}*.deb

ok "Successfully installed MVS v${VERSION}."

cd .. && rm -rf ./${PKG_MVS}
