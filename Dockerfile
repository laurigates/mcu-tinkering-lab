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

# Install Python development tools (base image is externally-managed)
RUN pip install --no-cache-dir --break-system-packages \
    pre-commit \
    pytest \
    pytest-cov \
    ruff \
    mypy \
    uv

# Create a non-root user matching the host UID/GID so build outputs in the
# bind-mounted workspace are owned by the developer, not root. See issue #318.
#
# Common collisions to handle:
#   * macOS `id -g` returns 20, which collides with Debian's `dialout` group.
#   * On Linux hosts, UID/GID 1000 may already be claimed by a base-image user.
# When the requested GID/UID already exists we reuse that group/user instead
# of failing the build.
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
        groupmod --new-name dev "$(getent group "${USER_GID}" | cut -d: -f1)" || true; \
    else \
        groupadd --gid "${USER_GID}" dev; \
    fi; \
    if getent passwd "${USER_UID}" >/dev/null; then \
        existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        usermod --login dev --home /home/dev --move-home --shell /bin/bash --gid "${USER_GID}" "${existing_user}" 2>/dev/null \
            || usermod --home /home/dev --shell /bin/bash --gid "${USER_GID}" "${existing_user}"; \
    else \
        useradd --create-home --uid "${USER_UID}" --gid "${USER_GID}" --shell /bin/bash dev; \
    fi; \
    install -d -o "${USER_UID}" -g "${USER_GID}" /home/dev /home/dev/.cache; \
    # ESP-IDF's entrypoint sources export.sh which writes into /opt/esp
    # (deactivate scripts, tool detection, ccache). Give dev ownership so the
    # toolchain can initialize without root.
    chown -R "${USER_UID}:${USER_GID}" /opt/esp /home/dev

# Set working directory
WORKDIR /workspace

# Configure git to trust the workspace directory for both root (image build)
# and the dev user (runtime).
RUN git config --global --add safe.directory /workspace
USER dev
RUN git config --global --add safe.directory /workspace

# Set environment variables
ENV HOME=/home/dev
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp
ENV PATH="${IDF_PATH}/tools:${PATH}"

# Default command
CMD ["/bin/bash"]
