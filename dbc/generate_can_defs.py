"""DBC to static CAN message definition header generator.

Parse a DBC file and auto-generates a header file with static definitions of CAN
bus messages and their signals (based on C based type definitions).

Follows clang-format style with 2-space indents.

Usage:
    ```shell
    python3 generate_can_defs.py path/to/your.dbc path/to/output  # Unix.
    ```

    ```shell
    py generate_can_defs.py path/to/your.dbc path/to/output  # WindowsOS.
    ```

Example:
    ```shell
    python3 generate_can_defs.py can_momentum.dbc can_dbc  # Unix.
    ```
    Creates "can_dbc.c" and "can_dbc.h" in the current directory based on
    can_momentum.dbc.
"""

import re
import sys
import argparse


def parse_dbc(filename: str):
    """Parse the DBC file for BO_ (messages) and SG_ (signals) lines."""
    messages = []
    current_msg = None

    # Regex to match a message (BO_ line):
    # Format: BO_ <msg_id> <msg_name>: <DLC> <transmitter>
    bo_pattern = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\S+)")

    # Regex to match a signal (SG_ line):
    # Format: SG_ <signal_name> : <start_bit>|<bit_length>@<byte_order><sign> (<scale>,<offset>) [<min>|<max>] "<unit>" <receiver>

    number = r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?"
    # Account for numbers with "e" for scientific notation.

    sg_pattern = re.compile(
        rf"^\s*SG_\s+(\w+)\s*:\s*"
        rf"(\d+)\|(\d+)@(\d)([+-])\s*"
        rf"\(\s*({number})\s*,\s*({number})\s*\)\s*"
        rf"\[\s*({number})\s*\|\s*({number})\s*\]\s*"
        rf'"([^"]*)"\s+(\S+)'
    )

    with open(filename, "r") as f:
        for line in f:
            line = line.rstrip()
            if line.startswith("BO_"):
                # Save previous message if exists.
                if current_msg is not None:
                    messages.append(current_msg)
                m = bo_pattern.match(line)
                if m:
                    msg_id = int(m.group(1))
                    msg_name = m.group(2)
                    dlc = int(m.group(3))
                    transmitter = m.group(4)
                    current_msg = {
                        "id": msg_id,
                        "name": msg_name,
                        "dlc": dlc,
                        "transmitter": transmitter,
                        "signals": [],
                    }
                else:
                    print("Failed to parse BO_ line:", line, file=sys.stderr)
            elif line.lstrip().startswith("SG_"):
                if current_msg is None:
                    # Signal without a message; skip it.
                    continue
                m = sg_pattern.match(line)
                if m:
                    signal_name = m.group(1)
                    start_bit = int(m.group(2))
                    bit_length = int(m.group(3))
                    dbc_byte_order = int(m.group(4))
                    # DBC: 1 = Intel (little-endian), 0 = Motorola (big-endian).

                    # Map DBC byte order to C based enum:
                    # CAN_LITTLE_ENDIAN = 0, CAN_BIG_ENDIAN = 1.
                    # if dbc_byte_order==1 (Intel), use CAN_LITTLE_ENDIAN;
                    # if dbc_byte_order==0, then CAN_BIG_ENDIAN.
                    if dbc_byte_order == 1:
                        byte_order = "CAN_LITTLE_ENDIAN"
                    else:
                        byte_order = "CAN_BIG_ENDIAN"

                    # The group 5 (sign) is typically '+', assume '+'.
                    scale = float(m.group(6))
                    offset = float(m.group(7))
                    min_value = float(m.group(8))
                    max_value = float(m.group(9))
                    unit = m.group(10)  # Can be an empty string.
                    receiver = m.group(11)

                    signal = {
                        "name": signal_name,
                        "start_bit": start_bit,
                        "bit_length": bit_length,
                        "byte_order": byte_order,
                        "scale": scale,
                        "offset": offset,
                        "min_value": min_value,
                        "max_value": max_value,
                        "unit": unit,
                    }
                    current_msg["signals"].append(signal)
                else:
                    print("Failed to parse SG_ line:", line, file=sys.stderr)
        # Append the final message, if any.
        if current_msg is not None:
            messages.append(current_msg)
    return messages


def generate_source(messages, output_filename: str):
    """Generate source file with extern array of CAN message definitions."""
    with open(f"{output_filename}.c", "w") as out:
        out.write(
            "/** Auto-generated CAN message definitions from DBC file. */\n\n"
        )
        out.write(f'#include "{output_filename}.h"\n\n')
        out.write("const can_message_t dbc_messages[] = {\n")
        for msg in messages:
            out.write("    {\n")
            out.write('        .name = "{0}",\n'.format(msg["name"]))
            out.write("        .message_id = {0},\n".format(msg["id"]))
            out.write("        .id_mask = 0xFFFFFFFF,\n")
            out.write("        .dlc = {0},\n".format(msg["dlc"]))
            out.write("        .rx_handler = 0,\n")
            out.write("        .tx_handler = 0,\n")
            out.write(
                "        .signal_count = {0},\n".format(len(msg["signals"]))
            )
            out.write("        .signals =\n            {\n")
            for sig in msg["signals"]:
                out.write("                {\n")
                out.write(
                    '                    .name = "{0}",\n'.format(sig["name"])
                )
                out.write(
                    "                    .start_bit = {0},\n".format(
                        sig["start_bit"]
                    )
                )
                out.write(
                    "                    .bit_length = {0},\n".format(
                        sig["bit_length"]
                    )
                )
                out.write(
                    "                    .byte_order = {0},\n".format(
                        sig["byte_order"]
                    )
                )
                out.write(
                    "                    .scale = {0}f,\n".format(sig["scale"])
                )
                out.write(
                    "                    .offset = {0}f,\n".format(
                        sig["offset"]
                    )
                )
                out.write(
                    "                    .min_value = {0}f,\n".format(
                        sig["min_value"]
                    )
                )
                out.write(
                    "                    .max_value = {0}f,\n".format(
                        sig["max_value"]
                    )
                )
                out.write("                },\n")
            out.write("            },\n")
            out.write("    },\n")
        out.write("};\n\n")
        out.write(
            "const int dbc_message_count = sizeof(dbc_messages) / sizeof(dbc_messages[0]);\n"
        )


def generate_header(output_filename: str):
    """Generate header file with appropriate extern definitions."""
    with open(f"{output_filename}.h", "w") as out:
        out.write(
            "/** Auto-generated CAN message definitions from DBC file. */\n\n"
        )
        out.write(f"#ifndef {output_filename.upper()}_H\n")
        out.write(f"#define {output_filename.upper()}_H\n\n")
        out.write('#include "can.h"\n\n')
        out.write("extern const can_message_t dbc_messages[];\n")
        out.write("extern const int dbc_message_count;\n\n")
        out.write(f"#endif // {output_filename.upper()}_H\n")


def main():
    # Get parse and generation arguments.
    parser = argparse.ArgumentParser(
        description="Generate code from a DBC file for CAN message definitions."
    )
    parser.add_argument("dbc_file", help="Path to the input DBC file")
    parser.add_argument("output_file", help="Path to the output header file")
    args = parser.parse_args()

    # Parse DBC.
    messages = parse_dbc(args.dbc_file)
    if not messages:  # Handle error.
        print("No messages found in the DBC file.", file=sys.stderr)
        sys.exit(1)

    # Generate header and source file.
    generate_header(args.output_file)
    generate_source(messages, args.output_file)

    # Output message.
    print(f'Files generated: "{args.output_file}.h", "{args.output_file}.c".')


if __name__ == "__main__":
    main()
