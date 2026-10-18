#!/usr/bin/env python3
import os
import sys
import subprocess
import glob
import re
import shutil
import threading
import time

# Color setup
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Persistent status bar
# Uses a terminal scroll region so idf output scrolls only in rows 1..N-1
# and the last row is permanently reserved — no cursor save/restore bleed.

_status_text = ""
_spinner_frames = "|/-\\"
_spinner_idx = 0
_spinner_stop = threading.Event()
_spinner_thread = None

def _rows():
    return shutil.get_terminal_size().lines

def _cols():
    return shutil.get_terminal_size().columns

def _spinner_loop():
    global _spinner_idx
    while not _spinner_stop.is_set():
        _spinner_idx = (_spinner_idx + 1) % len(_spinner_frames)
        _draw_status()
        time.sleep(0.1)

def init_status():
    """Reserve the bottom row and start the spinner thread."""
    global _spinner_thread
    if not sys.stdout.isatty():
        return
    rows = _rows()
    sys.stdout.write(
        f"\033[1;{rows - 1}r"  # scroll region = all rows except last
        f"\033[{rows - 1};1H"  # place cursor at last line of scroll region
    )
    sys.stdout.flush()
    _spinner_stop.clear()
    _spinner_thread = threading.Thread(target=_spinner_loop, daemon=True)
    _spinner_thread.start()

def set_status(text):
    """Update the status bar text (spinner redraws automatically)."""
    global _status_text
    _status_text = text

def _draw_status():
    """Render the status bar in the reserved bottom row."""
    if not sys.stdout.isatty():
        return
    rows, cols = _rows(), _cols()
    spin = _spinner_frames[_spinner_idx]
    bar_text = f"{spin} {_status_text}"
    pad = max(0, cols - len(bar_text))
    bar = f"{Colors.BOLD}{Colors.OKBLUE}{bar_text}{' ' * pad}{Colors.ENDC}"
    sys.stdout.write(
        f"\033[s"              # save cursor
        f"\033[{rows};1H"     # jump to reserved last row
        f"\033[2K"            # clear it
        f"{bar}"
        f"\033[u"             # restore cursor (stays in scroll region)
    )
    sys.stdout.flush()

def clear_status():
    """Stop the spinner, restore normal scroll region, and clear the status bar."""
    _spinner_stop.set()
    if not sys.stdout.isatty():
        return
    rows, cols = _rows(), _cols()
    sys.stdout.write(
        f"\033[r"             # reset scroll region to full terminal
        f"\033[{rows};1H"    # go to last row
        f"\033[2K"           # clear status bar
        "\n"                  # ensure we're past it
    )
    sys.stdout.flush()

def print_status(msg, color=Colors.OKBLUE):
    print(f"{color}{msg}{Colors.ENDC}")
    _draw_status()

def run_streamed(cmd, **kwargs):
    """Run a command, streaming stdout+stderr, redrawing the status bar each line."""
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        **kwargs
    )
    for line in proc.stdout:
        sys.stdout.write(line)
        _draw_status()
    proc.wait()
    return proc

# Hardware config discovery
def get_hw_configs():
    configs = []
    files = glob.glob("main/hwconf/**/hw_*.h", recursive=True)

    for f in files:
        hw_name = None
        hw_target = None
        try:
            with open(f, 'r') as header:
                content = header.read()
                name_match = re.search(r'#define\s+HW_NAME\s+"(.*?)"', content)
                if name_match:
                    hw_name = name_match.group(1)
                target_match = re.search(r'#define\s+HW_TARGET\s+"(.*?)"', content)
                if target_match:
                    hw_target = target_match.group(1)
            if hw_name and hw_target:
                configs.append({
                    'name': hw_name,
                    'target': hw_target,
                    'file': f
                })
        except Exception as e:
            print(f"Error parsing {f}: {e}")
    # Sort by target (SoC) first, then by name — groups same-chip builds together
    configs.sort(key=lambda x: (x['target'], x['name']))
    return configs

def build_target(config, output_dir, idx=0, total=0):
    build_dir = os.path.join("build", config['target'], re.sub(r'[^A-Za-z0-9_-]', '_', config['name']))
    shell = True if os.name == 'nt' else False

    print_status(f"\n========================================")
    print_status(f"Building: {config['name']} ({config['target']})")
    print_status(f"Config: {config['file']}")
    print_status(f"Dir: {build_dir}")
    print_status(f"========================================")

    cmd_base = [
        "idf.py", "-B", build_dir,
        f"-DIDF_TARGET={config['target'].split('_')[0]}",
        f"-DHW_NAME={config['name']}",
        f"-DSDKCONFIG={os.path.abspath(os.path.join(build_dir, 'sdkconfig'))}"
    ]

    # Build
    set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Building")
    print_status("--> Building...")
    res = run_streamed(cmd_base + ["build"], shell=shell)

    if res.returncode == 0:
        print_status(f"SUCCESS: {config['name']}", Colors.OKGREEN)

        # 3. Copy artifacts
        set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Copying artifacts")
        try:
            target_output_dir = os.path.join(output_dir, config['target'][0:7], config['name'])
            os.makedirs(target_output_dir, exist_ok=True)

            # Source paths
            src_bin = os.path.join(build_dir, "vesc_express.bin")
            src_boot = os.path.join(build_dir, "bootloader", "bootloader.bin")
            src_pt = os.path.join(build_dir, "partition_table", "partition-table.bin")

            if not os.path.exists(src_bin):
                raise FileNotFoundError(f"Missing artifact: {src_bin}")
            shutil.copy2(src_bin, os.path.join(target_output_dir, "vesc_express.bin"))
            print_status(f"--> Copied bin to {target_output_dir}")

            if not os.path.exists(src_boot):
                raise FileNotFoundError(f"Missing artifact: {src_boot}")
            shutil.copy2(src_boot, os.path.join(target_output_dir, "bootloader.bin"))
            print_status(f"--> Copied bootloader to {target_output_dir}")

            if not os.path.exists(src_pt):
                raise FileNotFoundError(f"Missing artifact: {src_pt}")
            shutil.copy2(src_pt, os.path.join(target_output_dir, "partition-table.bin"))
            print_status(f"--> Copied partition table to {target_output_dir}")

        except Exception as e:
            print_status(f"FAILED to copy artifacts for {config['name']}: {e}", Colors.FAIL)
            return False

        return True
    else:
        print_status(f"FAILED: {config['name']}", Colors.FAIL)
        return False

def main():
    if not os.path.exists("main/hwconf"):
        print_status("Error: main/hwconf directory not found", Colors.FAIL)
        sys.exit(1)

    # This is the firmware stub string
    res_firmwares_string = '        <file>TARGET_DESTINATION_DIRECTORY/TARGET_DESTINATION_FILENAME</file>\n'

    # This is the XML stub string
    resource_xml_stub_string = '''
<RCC>
   <qresource prefix="/res/firmwares_esp/">
REPLACEABLE_STRING
   </qresource>
</RCC>
'''

    # Declare an empty string
    res_string = ""

    # Prepare output directory
    output_dir = "build_output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print_status(f"Created output directory: {output_dir}")

    configs = get_hw_configs()
    total = len(configs)
    print_status(f"Found {total} hardware configurations.")

    success_count = 0
    failed_configs = []

    init_status()

    try:
        for idx, config in enumerate(configs, start=1):
            set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Starting")
            if build_target(config, output_dir, idx, total):
                success_count += 1

                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target'][0:7]).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/bootloader.bin")
                res_string = res_string + target_res_string
                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target'][0:7]).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/partition-table.bin")
                res_string = res_string + target_res_string
                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target'][0:7]).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/vesc_express.bin")
                res_string = res_string + target_res_string
            else:
                failed_configs.append(config['name'])

    except KeyboardInterrupt:
        clear_status()
        print_status("\nBuild interrupted by user.", Colors.WARNING)
        sys.exit(1)

    clear_status()

    print("\n" + "="*40)
    print(f"Build Summary: {success_count}/{total} Succeeded")
    print(f"Artifacts: {os.path.abspath(output_dir)}")

    with open(os.path.join(output_dir, 'res_fw.qrc'), 'w') as f:
        print(resource_xml_stub_string.replace("REPLACEABLE_STRING", res_string[:-1]), file=f)

    if failed_configs:
        print_status(f"Failed: {', '.join(failed_configs)}", Colors.FAIL)
        sys.exit(1)
    else:
        print_status("All builds successful!", Colors.OKGREEN)
        sys.exit(0)

if __name__ == "__main__":
    main()
