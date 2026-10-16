#!/usr/bin/env bash
#
# flash-openocd.sh — program the STM32F407IGHx via OpenOCD + CMSIS-DAP (DAP-Link).
#
#   ./flash-openocd.sh                build, then program + verify + reset
#   ./flash-openocd.sh --no-build     program the image that is already built
#   ./flash-openocd.sh --check        connection test only
#
#   OPENOCD_INTERFACE=interface/cmsis-dap.cfg OPENOCD_SPEED=1000 ./flash-openocd.sh
#   IMAGE=CubeMX/build/RM_Frame_C.hex ./flash-openocd.sh
#
# Uses the built-in OpenOCD scripts (interface/*.cfg, target/stm32f4x.cfg).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IFACE="${OPENOCD_INTERFACE:-interface/cmsis-dap.cfg}"
TARGET="${OPENOCD_TARGET:-target/stm32f4x.cfg}"
SPEED="${OPENOCD_SPEED:-1000}"          # kHz
IMAGE="${IMAGE:-$ROOT/CubeMX/build/RM_Frame_C.elf}"

DO_BUILD=1
CHECK=0

for arg in "$@"; do
    case "$arg" in
        --no-build) DO_BUILD=0 ;;
        --check)    CHECK=1; DO_BUILD=0 ;;
        -h|--help)  sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

command -v openocd >/dev/null || { echo "ERROR: openocd not found in PATH" >&2; exit 1; }

if (( DO_BUILD )); then
    echo ">> building"
    "$ROOT/build.sh"
fi

if (( ! CHECK )); then
    [[ -f "$IMAGE" ]] || { echo "ERROR: image not found: $IMAGE" >&2; exit 1; }
    echo ">> OpenOCD: $IFACE / $TARGET @ ${SPEED}kHz / $(basename "$IMAGE")"
    exec openocd \
        -f "$IFACE" \
        -c "transport select swd" \
        -c "adapter speed $SPEED" \
        -f "$TARGET" \
        -c "program \"$IMAGE\" verify reset exit"
else
    echo ">> OpenOCD: connection check ($IFACE / $TARGET @ ${SPEED}kHz)"
    exec openocd \
        -f "$IFACE" \
        -c "transport select swd" \
        -c "adapter speed $SPEED" \
        -f "$TARGET" \
        -c "init" -c "reset halt" -c "reg pc" -c "exit"
fi
