FROM ghcr.io/astral-sh/uv:0.13.5-python3.13-trixie-slim

WORKDIR /app
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy PYTHONUNBUFFERED=1
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --no-dev --no-install-project
COPY src ./src
RUN uv sync --frozen --no-dev

USER nobody
CMD ["/app/.venv/bin/robocar-glitchtip-bridge"]
