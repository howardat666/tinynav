#!/usr/bin/env bash
# Build pydbow3 for aarch64 (D-Robotics X5 inside the Looper camera) and stage
# it, together with every non-system shared library it needs, into an output
# directory ready to be pushed to the board.
#
#   ./build_aarch64.sh [OUT_DIR]        # default: /home/dm/looper/x5_aarch64
#
# Prerequisites: qemu-aarch64 binfmt registered (verify with
#   docker run --rm --platform linux/arm64 arm64v8/ubuntu:22.04 uname -m
# ) and the toolchain image built from Dockerfile.aarch64:
#   DOCKER_BUILDKIT=0 docker build --platform linux/arm64 \
#       -t tinynav-x5:dbow3-arm64 -f Dockerfile.aarch64 .
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-/home/dm/looper/x5_aarch64}"
IMAGE="${IMAGE:-tinynav-x5:dbow3-arm64}"

mkdir -p "${OUT_DIR}/lib"

# --user keeps the artifacts owned by the caller instead of root; HOME must be
# writable because pybind11/python otherwise try to write into /root.
docker run --rm --platform linux/arm64 \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "${HERE}:/shim:ro" -v "${OUT_DIR}:/out" \
    "${IMAGE}" bash -euo pipefail -c '
SUFFIX="$(python3 -c "import sysconfig; print(sysconfig.get_config_var(\"EXT_SUFFIX\"))")"
echo "arch=$(uname -m) python=$(python3 -V 2>&1) suffix=${SUFFIX}"

# Debian ships the pybind11 headers in /usr/include (package pybind11-dev) but
# not the "pybind11" python module, so `python3 -m pybind11 --includes` fails
# here; take the Python include dir from sysconfig instead.
PYINC="$(python3 -c "import sysconfig; print(sysconfig.get_paths()[\"include\"])")"
echo "pyinc=${PYINC}"

g++ -O3 -Wall -shared -std=c++14 -fPIC \
    "-I${PYINC}" \
    -I/usr/local/include -I/usr/include/opencv4 \
    /shim/pydbow3_shim.cpp \
    -o "/tmp/pydbow3${SUFFIX}" \
    -L/usr/local/lib -lDBoW3 -lopencv_core \
    -Wl,-rpath,"\$ORIGIN:\$ORIGIN/lib"

install -m 0755 "/tmp/pydbow3${SUFFIX}" "/out/pydbow3${SUFFIX}"

# Recursively stage every dependency that the board (a bare Ubuntu 22.04
# aarch64 userspace: glibc + libstdc++ only) will not already have.
python3 - <<PY
import os, re, shutil, subprocess

KEEP_ON_BOARD = re.compile(
    r"^(ld-linux-aarch64|libc|libm|libdl|libpthread|librt|libgcc_s|libstdc\+\+)\.so")

def deps(path):
    out = subprocess.run(["ldd", path], capture_output=True, text=True).stdout
    for line in out.splitlines():
        m = re.match(r"\s*(\S+)\s+=>\s+(/\S+)", line)
        if m:
            yield m.group(1), m.group(2)

# Walk the /tmp copy, not the one in /out: the extension carries an \$ORIGIN
# rpath, so on a re-run ldd would resolve to the previously staged copies and
# every shutil.copy2 would be a SameFileError.
seen, queue = set(), ["/tmp/pydbow3${SUFFIX}"]
while queue:
    cur = queue.pop()
    for soname, resolved in deps(cur):
        if soname in seen or KEEP_ON_BOARD.match(soname):
            continue
        if resolved.startswith("/out/"):
            continue
        seen.add(soname)
        real = os.path.realpath(resolved)
        shutil.copy2(real, "/out/lib/" + os.path.basename(real))
        # Keep the SONAME symlink chain so the loader finds it by name.
        for name in {soname, os.path.basename(resolved)}:
            link = "/out/lib/" + name
            if name != os.path.basename(real) and not os.path.exists(link):
                os.symlink(os.path.basename(real), link)
        queue.append(real)
        print("staged", soname, "->", real)
PY

echo "=== ldd pydbow3${SUFFIX} ==="
ldd "/out/pydbow3${SUFFIX}"
cp /src/DBoW3.commit /out/DBoW3.commit
'

echo
echo "Artifacts in ${OUT_DIR}:"
find "${OUT_DIR}" -maxdepth 2 -type f -o -maxdepth 2 -type l | sort
