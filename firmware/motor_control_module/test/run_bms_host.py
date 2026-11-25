"""Run BMS logic tests without ESP-IDF: python3 test/run_bms_host.py.

Only CAN transport is stubbed. Compile the actual decoder source with a copied
header so its can.hpp include resolves to the host stub, not ESP-IDF headers.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="terrascout-bms-") as directory:
    tmp = Path(directory)
    shutil.copy(root / "include/bms_interface.hpp", tmp / "bms_interface.hpp")
    (tmp / "can.hpp").write_text("""#pragma once
#include <cstddef>
#include <cstdint>
using esp_err_t = int;
constexpr int ESP_OK=0, ESP_ERR_INVALID_ARG=1, ESP_ERR_NOT_SUPPORTED=2;
class Can { public:
struct Frame { uint32_t id=0; bool extended=false; bool rtr=false;
uint8_t dlc=0; uint8_t data[8]{}; };
int receive(Frame&, uint32_t) { return 3; }
};
""")
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall",
                    "-Wextra", "-Werror", "-I", str(tmp),
                    str(root / "src/bms_interface.cpp"),
                    str(root / "test/bms_host.cpp"), "-o", str(tmp / "test")], check=True)
    subprocess.run([str(tmp / "test")], check=True)
print("BMS host tests passed")
