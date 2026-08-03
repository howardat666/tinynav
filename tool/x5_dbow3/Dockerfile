# Derived image: tinynav + DBoW3 + a minimal pydbow3 binding, so the CPU-only
# (plan 1 / 5a) retrieval path can run without TensorRT or a GPU.
FROM uniflexai/tinynav:latest

USER root

RUN git clone --depth 1 https://github.com/rmsalinas/DBow3.git /src/DBow3 \
    && cmake -S /src/DBow3 -B /src/DBow3/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DUSE_CONTRIB=OFF \
        -DBUILD_UTILS=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /src/DBow3/build -j"$(nproc)" \
    && cmake --install /src/DBow3/build \
    && ldconfig

# Upstream pyDBoW3 does not build against OpenCV 4.x, so use our own shim.
COPY pydbow3_shim.cpp /src/pydbow3_shim.cpp
# The venv has no python3-config, so take the extension suffix from sysconfig.
RUN set -eux; \
    SUFFIX="$(/opt/venv/bin/python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"; \
    g++ -O3 -Wall -shared -std=c++14 -fPIC \
        $(/opt/venv/bin/python3 -m pybind11 --includes) \
        -I/usr/local/include -I/usr/include/opencv4 \
        /src/pydbow3_shim.cpp \
        -o "/opt/venv/lib/python3.10/site-packages/pydbow3${SUFFIX}" \
        -L/usr/local/lib -lDBoW3 -lopencv_core; \
    ldconfig; \
    ls -la "/opt/venv/lib/python3.10/site-packages/pydbow3${SUFFIX}"

RUN /opt/venv/bin/python3 -c "import pydbow3; v=pydbow3.Vocabulary(); print('pydbow3 import OK', dir(pydbow3))"
