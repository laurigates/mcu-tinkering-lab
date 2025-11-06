# MCU Tinkering Lab - ESP-IDF Development Container
# Based on official Espressif ESP-IDF image

FROM espressif/idf:v5.4

LABEL maintainer="MCU Tinkering Lab"
LABEL description="ESP32 development environment with tooling"

# Install additional development tools
RUN apt-get update && apt-get install -y \
    clang-format \
    cppcheck \
    git \
    make \
    cmake \
    ninja-build \
    python3-pip \
    python3-venv \
    vim \
    nano \
    curl \
    wget \
    # For serial port access
    minicom \
    screen \
    # For debugging
    gdb \
    gdb-multiarch \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python development tools
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir \
    pre-commit \
    pytest \
    pytest-cov \
    ruff \
    mypy \
    uv

# Set working directory
WORKDIR /workspace

# Configure git to trust the workspace directory (for pre-commit)
RUN git config --global --add safe.directory /workspace

# Set environment variables
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp
ENV PATH="${IDF_PATH}/tools:${PATH}"

# Default command
CMD ["/bin/bash"]
