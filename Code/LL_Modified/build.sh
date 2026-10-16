#!/usr/bin/env bash
#
# Debug build wrapper for the LL_Modified copy.
#
#   ./build.sh            # regenerate CubeMX/Makefile.gen and build
#   ./build.sh clean      # remove build outputs
#   ./build.sh -j8        # extra args are forwarded to make
#
# Toolchain selection: STM32CubeCLT GNU tools are preferred (stable, matches
# what CubeIDE ships); set GCC_PATH to override, or GCC_PATH= to force the
# toolchain found in PATH.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CUBEMX="$ROOT/CubeMX"

# --- pick a toolchain --------------------------------------------------------
ST_TOOLCHAIN="/opt/stm32cubeclt/GNU-tools-for-STM32/bin"
if [[ -z "${GCC_PATH+set}" ]]; then
    if [[ -x "$ST_TOOLCHAIN/arm-none-eabi-gcc" ]]; then
        GCC_PATH="$ST_TOOLCHAIN"
    else
        GCC_PATH=""
    fi
fi

if [[ -n "$GCC_PATH" ]]; then
    CC="$GCC_PATH/arm-none-eabi-gcc"
else
    CC="$(command -v arm-none-eabi-gcc)"
fi
echo ">> toolchain: $CC ($("$CC" -dumpversion))"
[[ -n "$GCC_PATH" ]] && echo ">> GCC_PATH=$GCC_PATH"

# --- post-generation patches + regenerated Makefile -------------------------
python3 "$ROOT/tools/patch_generated.py"
python3 "$ROOT/tools/gen_makefile.py"

# --- build -------------------------------------------------------------------
mkdir -p "$CUBEMX"
make -C "$CUBEMX" -f Makefile.gen GCC_PATH="$GCC_PATH" "$@"
