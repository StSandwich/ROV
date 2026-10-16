#!/usr/bin/env python3
"""
Post-generation patches for the LL_Modified debug copy.

CubeMX regenerates Src/main.c on every code generation.  Two things in the
generated file are incompatible with a GCC/Makefile build *and* with this
project's existing architecture:

  1. USER CODE PTD still contains the Keil/ARMCC semihosting retargeting
     (rt_sys.h, _sys_*, and a hard ``#error Unsupported compiler``).
     -> wrap it in ``#if defined(__ARMCC_VERSION)``; for GCC the retargeting
        comes from the generated Src/syscalls.c + Src/sysmem.c.

  2. CubeMX injects a fresh ``int main(void)`` (ProjectManager.NoMain=false),
     but this project's real ``main()`` lives in
     userCode/devices/Src/Usermain.cpp.
     -> disable the generated copy so there is only one ``main`` symbol.

The script is idempotent: running it twice changes nothing.
The proper long-term fix is to set ``ProjectManager.NoMain=true`` in the .ioc
and to guard the Keil block in the real source tree.

Run:  python3 tools/patch_generated.py
"""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_C = os.path.join(ROOT, "Src", "main.c")
IT_C = os.path.join(ROOT, "Src", "stm32f4xx_it.c")

# IRQ handlers that the application implements itself (in userCode/) and that
# must therefore not be emitted by the generated stm32f4xx_it.c.
# DMA2_Stream0 == SPI1_RX, handled by IMU::imu.DMA_IT_Handle() in IMU.cpp.
OVERRIDDEN_IRQ = ["DMA2_Stream0_IRQHandler"]

BEGIN = "/* USER CODE BEGIN PTD */"
END = "/* USER CODE END PTD */"
GUARD_OPEN = "#if defined(__ARMCC_VERSION)  /* Keil/ARMCC only - patched */"
GUARD_MID = "#else\n/* GCC: retargeting is provided by Src/syscalls.c + Src/sysmem.c */"
GUARD_CLOSE = "#endif /* __ARMCC_VERSION */"
MAIN_OFF = "#if 0  /* patched: real main() is userCode/devices/Src/Usermain.cpp */"


def wrap_ptd(text: str) -> str:
    if "Keil/ARMCC only - patched" in text:
        return text  # already patched
    i = text.index(BEGIN) + len(BEGIN)
    j = text.index(END, i)
    inner = text[i:j]
    patched = f"\n{GUARD_OPEN}\n{inner}{GUARD_MID}\n{GUARD_CLOSE}\n"
    return text[:i] + patched + text[j:]


def _match_brace(text: str, open_idx: int) -> int:
    """Return index of the '}' closing the '{' at open_idx (no comments/strings
    with braces are expected in a generated main())."""
    depth = 0
    for k in range(open_idx, len(text)):
        if text[k] == "{":
            depth += 1
        elif text[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    raise RuntimeError("unbalanced braces in main()")


def disable_main(text: str) -> str:
    if MAIN_OFF in text:
        return text  # already patched
    marker = "int main(void)"
    i = text.index(marker)
    brace = text.index("{", i)
    end = _match_brace(text, brace)
    return text[:i] + MAIN_OFF + "\n" + text[i:end + 1] + "\n#endif\n" + text[end + 1:]


def disable_irq_handlers(text: str, names: list[str]) -> str:
    """Wrap generated IRQ handlers overridden by user code in #if 0 ... #endif."""
    for name in names:
        if f"#if 0  /* patched: {name}" in text:
            continue  # already patched
        sig = f"void {name}(void)"
        if sig not in text:
            continue
        i = text.index(sig)
        brace = text.index("{", i)
        end = _match_brace(text, brace)
        guard = f"#if 0  /* patched: {name} is implemented in userCode */"
        text = text[:i] + guard + "\n" + text[i:end + 1] + "\n#endif\n" + text[end + 1:]
    return text


def patch_file(path: str, fn) -> bool:
    """Apply fn(text)->text to a generated file, preserving CRLF. Returns changed."""
    with open(path, "rb") as fh:
        raw = fh.read().decode("utf-8")
    crlf = "\r\n" in raw
    text = raw.replace("\r\n", "\n")
    new = fn(text)
    if new == text:
        return False
    out = new.replace("\n", "\r\n") if crlf else new
    with open(path, "wb") as fh:
        fh.write(out.encode("utf-8"))
    return True


def main() -> int:
    if patch_file(MAIN_C, lambda t: disable_main(wrap_ptd(t))):
        print("patched Src/main.c (PTD guard + generated main() disabled)")
    else:
        print("Src/main.c already patched")

    if patch_file(IT_C, lambda t: disable_irq_handlers(t, OVERRIDDEN_IRQ)):
        print(f"patched Src/stm32f4xx_it.c (disabled {', '.join(OVERRIDDEN_IRQ)})")
    else:
        print("Src/stm32f4xx_it.c already patched")
    return 0


if __name__ == "__main__":
    sys.exit(main())
