#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd "$(dirname "$0")" && pwd)
UV_BIN=uv
if ! command -v uv >/dev/null 2>&1; then
    UV_BIN="$SCRIPT_DIR/.cache/docs-preview/uv/uv"
fi

if [ "$UV_BIN" != uv ] && [ ! -x "$UV_BIN" ]; then
    echo "uv was not found. Installing it for this repository..."
    INSTALLER=$(mktemp)
    trap 'rm -f "$INSTALLER"' EXIT

    if command -v curl >/dev/null 2>&1; then
        curl -LsSf https://astral.sh/uv/install.sh -o "$INSTALLER"
    elif command -v wget >/dev/null 2>&1; then
        wget -qO "$INSTALLER" https://astral.sh/uv/install.sh
    else
        echo "Install curl or wget, then run this script again." >&2
        exit 1
    fi

    if ! UV_UNMANAGED_INSTALL="$SCRIPT_DIR/.cache/docs-preview/uv" \
        sh "$INSTALLER"; then
        echo "uv installation failed. Check your internet connection." >&2
        exit 1
    fi
    rm -f "$INSTALLER"
    trap - EXIT
fi

if ! command -v "$UV_BIN" >/dev/null 2>&1; then
    echo "uv installation did not create the expected executable." >&2
    exit 1
fi

exec "$UV_BIN" run --isolated --no-project --python 3.13 \
    --with-requirements "$SCRIPT_DIR/docs/requirements.txt" \
    python -m mkdocs serve --config-file "$SCRIPT_DIR/mkdocs.yml" \
    --dev-addr 127.0.0.1:8000 "$@"
