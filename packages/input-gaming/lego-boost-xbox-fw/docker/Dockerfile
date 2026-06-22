# Build container for Pybricks Move Hub firmware (feasibility spike).
#
# Provides the arm-none-eabi cross-compiler + Python/poetry that the Pybricks
# `make` build needs. The pybricks-micropython source is bind-mounted at /src by
# the justfile (it is NOT copied into the image).
#
# Toolchain version caveat: Debian's gcc-arm-none-eabi may differ from the
# version Pybricks pins in CI. If `make` fails on a toolchain mismatch, check
# external/pybricks-micropython/.github/workflows/ for the expected GCC version
# and either install that release tarball here or match it.
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
        build-essential \
        git \
        python3 \
        python3-pip \
        python3-venv \
        pipx \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Poetry manages the Python tools the build invokes (code generation, packaging).
ENV PIPX_HOME=/opt/pipx PIPX_BIN_DIR=/usr/local/bin
RUN pipx install poetry

WORKDIR /src
CMD ["bash"]
