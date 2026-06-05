#!/usr/bin/env python3
"""
Generate docs/rhsp-protocol.json from the authoritative message definitions in
src/rhsp/internal/messages.py.

The structural parts (command ids, packet/payload field names, field byte
widths, field order, request->response mapping) are extracted by *introspecting*
the live message classes, so they cannot drift from the implementation. A
curated SEMANTICS overlay adds information that is not encoded in messages.py
(signedness, fixed-point scaling, units, enum references, channel counts) and is
sourced from the higher-level modules (motors.py, servo.py, adc.py, color.py,
distance.py, imu.py). SEQUENCING recipes are distilled from the example scripts
in test/.

Run:  .venv/bin/python docs/generate_protocol_json.py
Output: docs/rhsp-protocol.json
"""
import json
import os
import sys

# Make the (reference) package importable when run from the repo root.
# The original implementation now lives under vendor/rhsp; this generator
# introspects it until it is inverted to read the new src/rhsp catalogue.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vendor"))

import rhsp.internal.messages as M


# --------------------------------------------------------------------------
# Curated, per-field semantics not encoded in messages.py.
# Keyed by "PacketName.fieldName". Any field without an entry is an unsigned
# little-endian integer with no special scaling.
# --------------------------------------------------------------------------
SEMANTICS = {
    # Motor target / feedback
    "SetMotorConstantPower.powerLevel": {"signed": True, "range": [-32767, 32767],
        "description": "Open-loop power. Full-scale magnitude is 2^15-1."},
    "GetMotorConstantPower_RSP.powerLevel": {"signed": True},
    "SetMotorTargetVelocity.velocity": {"signed": True, "unit": "encoder_counts_per_second"},
    "GetMotorTargetVelocity_RSP.velocity": {"signed": True, "unit": "encoder_counts_per_second"},
    "SetMotorTargetPosition.position": {"signed": True, "unit": "encoder_counts"},
    "GetMotorTargetPosition_RSP.targetPosition": {"signed": True, "unit": "encoder_counts"},
    "GetMotorEncoderPosition_RSP.currentPosition": {"signed": True, "unit": "encoder_counts",
        "description": "32-bit signed two's-complement encoder count."},
    "SetMotorChannelMode.motorMode": {"enum": "MotorMode"},
    "GetMotorChannelMode_RSP.motorChannelMode": {"enum": "MotorMode"},
    "SetMotorChannelMode.floatAtZero": {"enum": "ZeroPowerBehavior"},
    "GetMotorChannelMode_RSP.floatAtZero": {"enum": "ZeroPowerBehavior"},
    "SetMotorChannelCurrentAlertLevel.currentLimit": {"unit": "mA"},
    "GetMotorChannelCurrentAlertLevel_RSP.currentLimit": {"unit": "mA"},
    # PID
    "SetMotorPIDCoefficients.mode": {"enum": "ClosedLoopMode"},
    "GetMotorPIDCoefficients.mode": {"enum": "ClosedLoopMode"},
    "SetMotorPIDCoefficients.p": {"signed": True, "fixed_point": "Q16",
        "description": "Coefficient * 65536, 32-bit."},
    "SetMotorPIDCoefficients.i": {"signed": True, "fixed_point": "Q16"},
    "SetMotorPIDCoefficients.d": {"signed": True, "fixed_point": "Q16"},
    "GetMotorPIDCoefficients_RSP.p": {"signed": True, "fixed_point": "Q16"},
    "GetMotorPIDCoefficients_RSP.i": {"signed": True, "fixed_point": "Q16"},
    "GetMotorPIDCoefficients_RSP.d": {"signed": True, "fixed_point": "Q16"},
    # Servo / PWM
    "SetServoConfiguration.framePeriod": {"unit": "us", "description": "Servo frame period; typically 20000 (50 Hz)."},
    "GetServoConfiguration_RSP.framePeriod": {"unit": "us"},
    "SetServoPulseWidth.pulseWidth": {"unit": "us", "range": [500, 2500]},
    "GetServoPulseWidth_RSP.pulseWidth": {"unit": "us"},
    "SetPWMConfiguration.framePeriod": {"unit": "us"},
    "GetPWMConfiguration_RSP.framePeriod": {"unit": "us"},
    "SetPWMPulseWidth.pulseWidth": {"unit": "us"},
    # ADC
    "GetADC.adcChannel": {"enum": "ADCChannel"},
    "GetADC.rawMode": {"description": "0 = scaled/engineering units, nonzero = raw counts."},
    "GetADC_RSP.adcValue": {"description": "Reading; engineering units are mA/mV depending on channel (see ADCChannel)."},
    # DIO
    "SetSingleDIOOutput.value": {"description": "0 or 1."},
    "SetAllDIOOutputs.values": {"description": "Bitmask, one bit per DIO pin (8 pins)."},
    "GetAllDIOInputs_RSP.inputValues": {"description": "Bitmask, one bit per DIO pin."},
    "SetDIODirection.directionOutput": {"enum": "DIODirection"},
    "GetDIODirection_RSP.directionOutput": {"enum": "DIODirection"},
    # I2C
    "I2CConfigureChannel.speedCode": {"enum": "I2CSpeedCode"},
    "I2CConfigureQuery_RSP.speedCode": {"enum": "I2CSpeedCode"},
    "I2CBlockReadConfig.readInterval_ms": {"unit": "ms"},
    "IMUBlockReadConfig.readInterval_ms": {"unit": "ms"},
    # LED
    "SetModuleLEDColor.redPower": {"range": [0, 255]},
    "SetModuleLEDColor.greenPower": {"range": [0, 255]},
    "SetModuleLEDColor.bluePower": {"range": [0, 255]},
    # Discovery / status
    "Discovery_RSP.parent": {"description": "Parent module address; 0/255 for a USB-attached parent hub."},
    "GetModuleStatus.clearStatus": {"description": "Nonzero clears the latched status after reading."},
    "GetModuleStatus_RSP.motorAlerts": {"description": "Per-motor alert bitmask."},
    # Bulk monotonic timestamps
    "GetBulkInputData_RSP.mototonicTime": {"unit": "ms", "description": "Monotonic timestamp (field name misspelled in source)."},
    "GetBulkMotorData_RSP.monotonicTime": {"unit": "ms"},
    "GetBulkADCData_RSP.monotonicTime": {"unit": "ms"},
    "GetBulkI2CData_RSP.monotonicTime": {"unit": "ms"},
    "GetBulkServoData_RSP.monotonicTime": {"unit": "ms"},
}

# Per-command notes (behavioral, not field-level).
COMMAND_NOTES = {
    "Discovery": "Broadcast to address 255. The hub and any daisy-chained child "
                 "modules each emit a Discovery_RSP; collect replies until the "
                 "input buffer drains. Must be the first transaction.",
    "KeepAlive": "Resets the hub watchdog. Must be sent at least every ~2.5 s or "
                 "the hub enters fail-safe and disables all outputs.",
    "FailSafe": "Immediately puts the hub in fail-safe (outputs disabled).",
    "QueryInterface": "Returns the base packetID and command count for a named "
                      "interface. Send interfaceName='DEKA' to learn the runtime "
                      "base id of the I/O command block (0x1000 in this firmware).",
    "SetNewModuleAddress": "Changes the module's address; all subsequent commands "
                           "must target the new address.",
    "GetBulkInputData": "Single-transaction snapshot of all digital/analog/motor/"
                        "servo/I2C state. Preferred over many individual gets.",
    "SetMotorChannelMode": "Must be set before enabling a motor. Mode selects the "
                           "control loop (power/velocity/position/current).",
    "I2CBlockReadConfig": "Configures the hub to autonomously poll an I2C register "
                          "block; cached results are then read cheaply via "
                          "GetBulkI2CData. NOTE: the Python helper has a bug (writes "
                          "to the message object, not its payload).",
}

ENUMS = {
    "MotorMode": {
        "0": "CONSTANT_POWER", "1": "CONSTANT_VELOCITY",
        "2": "POSITION_TARGET", "3": "CONSTANT_CURRENT",
    },
    "ZeroPowerBehavior": {"0": "BRAKE_AT_ZERO", "1": "FLOAT_AT_ZERO"},
    "ClosedLoopMode": {
        "1": "VELOCITY", "2": "POSITION", "3": "CURRENT",
        "_note": "Used as the 'mode' selector for Set/GetMotorPIDCoefficients.",
    },
    "DIODirection": {"0": "INPUT", "1": "OUTPUT"},
    "ADCChannel": {str(getattr(M.ADCChannel, n)): n
                   for n in dir(M.ADCChannel) if not n.startswith("_")},
    "LEDColor": {str(getattr(M.LEDColor, n)): n
                 for n in dir(M.LEDColor) if not n.startswith("_")},
    "I2CSpeedCode": {"0": "STANDARD_100KHZ", "1": "FAST_400KHZ",
                     "_note": "Values per REV firmware; confirm against librhsp."},
    # Source: DuckLynx info/RHSP.md (stock-firmware semantics). The Python layer
    # only surfaces the raw byte.
    "NackCode": {
        "0-9": "Parameter #N out of range (N = code)",
        "10-17": "GPIO #(code-10) not configured for output",
        "18": "No GPIO pins configured for output",
        "20-27": "GPIO #(code-20) not configured for input",
        "28": "No GPIO pins configured for input",
        "30": "Servo not fully configured before enabled",
        "31": "Battery voltage too low to run servo",
        "40": "I2C master busy (command rejected)",
        "41": "I2C operation in progress (poll again)",
        "42": "I2C no results pending",
        "43": "I2C query mismatch",
        "44": "I2C timeout - SDA stuck",
        "45": "I2C timeout - SCK stuck",
        "46": "I2C timeout",
        "50": "Motor not fully configured for mode before enabled",
        "51": "Command not valid for selected motor mode",
        "52": "Battery voltage too low to run motor",
        "253": "Command implementation pending",
        "254": "Command routing error",
        "255": "Packet Type ID unknown",
        "_note": "Codes 19, 29, 32-39, 47-49, 53-59 are reserved.",
    },
    # GetModuleStatus response byte 0 (bit -> meaning).
    "ModuleStatusBits": {
        "0": "KeepAliveTimeout", "1": "DeviceReset", "2": "FailSafe",
        "3": "ControllerOverTemp", "4": "BatteryLow", "5": "HIBFault",
        "_note": "Bits 6-7 reserved. Source: DuckLynx info/RHSP.md.",
    },
    # GetModuleStatus response byte 1 (motor alerts).
    "MotorStatusBits": {
        "0": "Motor0LostCounts", "1": "Motor1LostCounts",
        "2": "Motor2LostCounts", "3": "Motor3LostCounts",
        "4": "Motor0DriverOverheat", "5": "Motor1DriverOverheat",
        "6": "Motor2DriverOverheat", "7": "Motor3DriverOverheat",
    },
}

# DEKA function ids >= 0x31 differ between this Python package and REV's stock
# firmware / librhsp. Source: DuckLynx info/RHSP.md, confirmed against librhsp
# deviceControl.c / motor.c. See docs/RHSP-Protocol.md section 4.6.
FIRMWARE_COMMAND_MAP_DIVERGENCE = {
    "_note": "Function ids 0x00-0x30 agree across implementations. From 0x31 "
             "onward the stock firmware/librhsp map differs from this package's "
             "messages.py. Resolve DEKA ids dynamically via QueryInterface and "
             "validate high ids against the target hub.",
    "agree_through_index": 48,
    "stock_firmware": {
        "49": "FTDI_RESET_CONTROL", "50": "FTDI_RESET_QUERY",
        "51": "SET_MOTOR_PIDF_COEFFICIENTS", "52": "I2C_WRITE_READ_MULTIPLE_BYTES",
        "53": "GET_MOTOR_PIDF_COEFFICIENTS", "54": "I2C_TRANSACTION",
        "55": "I2C_QUERY_TRANSACTION", "56": "SET_BULK_OUTPUT_DATA",
        "57": "READ_VERSION",
    },
    "python_rhsp_package": {
        "49": "GetBulkPIDData", "50": "I2CBlockReadConfig",
        "51": "I2CBlockReadQuery", "52": "I2CWriteReadMultipleBytes",
        "53": "IMUBlockReadConfig", "54": "IMUBlockReadQuery",
        "55": "GetBulkMotorData", "56": "GetBulkADCData",
        "57": "GetBulkI2CData", "64": "GetBulkServoData",
    },
}

# Sequencing recipes distilled from test/*.py and module.init_periphs().
SEQUENCING = {
    "session_bringup": {
        "description": "Required order to go from no connection to a usable module.",
        "steps": [
            "Enumerate serial ports (USB serial number starts with 'D').",
            "Open the port at 460800 8N1, no flow control.",
            "Send Discovery (broadcast, dest=255); build a Module per Discovery_RSP.",
            "Optionally QueryInterface('DEKA') to confirm the I/O command base id.",
            "Initialize peripherals (see module_init_periphs).",
            "Start a KeepAlive heartbeat (>= every ~2.5 s) for as long as outputs are active.",
        ],
    },
    "module_init_periphs": {
        "description": "What Module.init_periphs() does, per hardware block.",
        "steps": [
            "For each of 4 motor channels: SetMotorChannelMode(channel, mode=0, floatAtZero=1) then SetMotorConstantPower(channel, 0).",
            "Create 4 I2C channel handles (no traffic until a device is added).",
            "Create 8 DIO pin handles.",
            "For each of 6 servo channels: SetServoConfiguration(channel, framePeriod=20000).",
            "Create 4 ADC pin handles.",
        ],
    },
    "motor_open_loop_power": {
        "description": "Drive a motor with constant power (test_motor.py).",
        "steps": [
            "SetMotorChannelMode(channel, mode=0 CONSTANT_POWER, floatAtZero=1)",
            "SetMotorConstantPower(channel, 0)",
            "SetMotorChannelEnable(channel, 1)",
            "SetMotorConstantPower(channel, power)  # power in [-32767, 32767]",
            "KeepAlive() periodically",
        ],
    },
    "motor_closed_loop_velocity": {
        "description": "Run a motor at a target velocity (test_motor_velocity.py).",
        "steps": [
            "SetMotorChannelMode(channel, mode=0, floatAtZero=1); SetMotorConstantPower(channel, 0)  # init()",
            "SetMotorChannelEnable(channel, 1)",
            "SetMotorChannelMode(channel, mode=1 CONSTANT_VELOCITY, floatAtZero=1)",
            "SetMotorTargetVelocity(channel, velocity)",
            "Read velocity via GetBulkMotorData (motorNVelocity is 16-bit signed)",
            "KeepAlive() periodically",
        ],
    },
    "servo": {
        "description": "Position a servo (test_servo.py).",
        "steps": [
            "SetServoConfiguration(channel, framePeriod=20000)",
            "SetServoPulseWidth(channel, 1500)  # center, microseconds",
            "SetServoEnable(channel, 1)",
            "SetServoPulseWidth(channel, pw)  # 500..2500 us; angle 0..180 -> 500 + angle*2000/180",
        ],
    },
    "i2c_color_sensor_v2_apds9960": {
        "description": "Bring up the REV color sensor (color.py ColorSensor).",
        "steps": [
            "channel.addColorSensor()  # I2C addr 0x39",
            "writeByte(COMMAND_BIT|ENABLE_REG); writeByte(0x07)  # power on + ADC + wait",
            "writeByte(COMMAND_BIT|ATIME_REG); writeByte(0xFF)",
            "writeByte(COMMAND_BIT|PPULSE_REG); writeByte(0x08)",
            "Verify device id == 0x60 via ID_REG",
            "Read color: writeByte(COMMAND_BIT|MULTI_BYTE_BIT|xDATA_REG); readMultipleBytes(2)",
        ],
    },
    "i2c_distance_sensor_vl53l0x": {
        "description": "Bring up the REV 2m distance sensor (distance.py).",
        "steps": [
            "channel.addDistanceSensor()  # I2C addr 0x29",
            "initialize(): data-init register sequence, read stop_variable, set signal-rate limit,",
            "  load SPAD config, load default tuning register map, set GPIO interrupt,",
            "  performSingleRefCalibration(0x40) then performSingleRefCalibration(0x00),",
            "  setTimeout(200), startContinuous().",
            "readRangeContinuousMillimeters(): poll RESULT_INTERRUPT_STATUS until ready, read range, clear interrupt.",
        ],
    },
    "imu_block_read": {
        "description": "Autonomous IMU polling (BNO055).",
        "steps": [
            "IMUBlockReadConfig(startRegister, numberOfBytes, readInterval_ms)",
            "Read cached block via GetBulkI2CData (imuBlock field) or IMUBlockReadQuery",
        ],
    },
}


def field_list(payload):
    """Return [{name, bytes, offset, ...semantics}] in wire order for a payload."""
    members = []
    for name, val in vars(payload).items():
        if isinstance(val, M.REVBytes):
            members.append((val.memberOrder, name, val.numBytes))
    members.sort()
    fields = []
    offset = 0
    for _order, name, nbytes in members:
        f = {"name": name, "bytes": nbytes, "offset": offset}
        offset += nbytes
        fields.append((f, name))
    return fields


def annotate(fields, packet_name):
    out = []
    for f, name in fields:
        sem = SEMANTICS.get(f"{packet_name}.{name}")
        if sem:
            f.update(sem)
        out.append(f)
    return out


def build():
    ACK = M.MsgNum.ACK
    commands = []
    responses = []

    for cmd_id, info in sorted(M.printDict.items()):
        name = info["Name"]
        packet_cls = info["Packet"]
        # Some RSP entries use a buggy 'Response ' key (trailing space) in messages.py.
        resp = info.get("Response", info.get("Response "))
        pkt = packet_cls()
        payload = annotate(field_list(pkt.payload), name)

        is_response = name.endswith("_RSP") or name in ("ACK", "NACK")
        entry = {
            "name": name,
            "id": cmd_id,
            "id_hex": "0x%04X" % cmd_id,
            "group": ("system" if cmd_id < M.MsgNum.DekaInterfacePrefix or cmd_id >= 0x7F00
                      else "deka") if not is_response else "response",
            "payload": payload,
            "payload_bytes": sum(f["bytes"] for f in payload),
        }
        if is_response:
            responses.append(entry)
            continue

        # Map the reply.
        if resp is None:
            entry["reply"] = None
        elif resp == ACK:
            entry["reply"] = {"kind": "ack", "command": "ACK", "id": ACK, "id_hex": "0x%04X" % ACK}
        else:
            rinfo = M.printDict.get(resp, {})
            entry["reply"] = {
                "kind": "response",
                "command": rinfo.get("Name"),
                "id": resp,
                "id_hex": "0x%04X" % resp,
            }
        if name in COMMAND_NOTES:
            entry["notes"] = COMMAND_NOTES[name]
        commands.append(entry)

    doc = {
        "_about": "Structured description of the REV Hub Serial Protocol (RHSP / "
                  "DEKA interface) as implemented by the rhsp Python package. "
                  "Generated by docs/generate_protocol_json.py from "
                  "src/rhsp/internal/messages.py. Field structure is introspected "
                  "from the live classes; semantics/sequencing are curated.",
        "protocol": {
            "name": "REV Hub Serial Protocol (RHSP)",
            "interface": "DEKA",
            "transaction_model": "half-duplex request/response, one outstanding at a time",
            "see_also": "docs/RHSP-Protocol.md",
        },
        "link_layer": {
            "transport": "USB CDC virtual serial port",
            "baud": 460800, "data_bits": 8, "parity": "none", "stop_bits": 1,
            "flow_control": "none",
            "port_discovery": "USB hwid SER= field; hub serial numbers start with 'D'.",
            "topology": "Controller <-> parent hub over USB (UART0). Child hubs "
                        "connect to the parent over RS485 (UART1) and are reached "
                        "via the parent's packet forwarding. Same protocol on both "
                        "legs. Source: DuckLynx info/RHSP.md.",
        },
        "framing": {
            "frame_bytes": [0x44, 0x4B],
            "frame_bytes_ascii": "DK",
            "byte_order": "little-endian",
            "max_payload_size": M.PAYLOAD_MAX_SIZE,
            "max_payload_size_note": "This Python package caps payload at "
                                     "PAYLOAD_MAX_SIZE=128; REV librhsp allows up "
                                     "to 512. Size receive buffers for 512.",
            "header_fields": [
                {"name": "frameBytes", "bytes": 2, "offset": 0, "value": "0x44 0x4B"},
                {"name": "packetLength", "bytes": 2, "offset": 2,
                 "description": "Total packet size in bytes incl. frame, header, payload, checksum."},
                {"name": "destination", "bytes": 1, "offset": 4,
                 "description": "Target module address; 0xFF (255) = broadcast."},
                {"name": "source", "bytes": 1, "offset": 5,
                 "description": "Always 0x00 for the controller; the hub's address on a reply."},
                {"name": "messageNumber", "bytes": 1, "offset": 6,
                 "description": "Per spec must be >= 1 (never 0); starts at 1, wraps to 1 on overflow."},
                {"name": "referenceNumber", "bytes": 1, "offset": 7,
                 "description": "On a response, echoes the request's messageNumber."},
                {"name": "packetType", "bytes": 2, "offset": 8,
                 "description": "Command id; bit 15 = response flag."},
            ],
            "header_bytes": 8,
            "payload_offset": 10,
            "checksum": {
                "size_bytes": 1, "position": "last byte",
                "algorithm": "8-bit sum of all bytes from frame start through end of payload, mod 256",
                "on_bad_checksum": "Hub sends no reply and does not reset the keep-alive timeout.",
            },
            "min_packet_bytes": 11,
        },
        "constants": {
            "response_bit": M.RESPONSE_BIT,
            "response_bit_hex": "0x%04X" % M.RESPONSE_BIT,
            "deka_interface_prefix": M.MsgNum.DekaInterfacePrefix,
            "deka_interface_prefix_hex": "0x%04X" % M.MsgNum.DekaInterfacePrefix,
            "broadcast_address": 255,
            "system_command_base_hex": "0x7F01",
            "response_id_rule": "A typed response id = response_bit | request_id (e.g. GetADC 0x1007 -> 0x9007). ACK/NACK keep their own ids.",
        },
        "enums": ENUMS,
        "counts": {
            "motor_channels": 4, "servo_channels": 6, "dio_pins": 8,
            "adc_channels": 4, "i2c_channels": 4,
        },
        "keep_alive_interval_ms": 2500,
        "keep_alive_note": "Hub enters fail-safe (outputs disabled) after 2500 ms "
                           "with no valid packet. Any valid packet resets the "
                           "timeout, even one that is NACK'd; bad-checksum/no-magic "
                           "packets do not. Source: DuckLynx info/RHSP.md.",
        "firmware_command_map_divergence": FIRMWARE_COMMAND_MAP_DIVERGENCE,
        "commands": commands,
        "responses": responses,
        "sequencing": SEQUENCING,
    }
    return doc


if __name__ == "__main__":
    doc = build()
    out_path = os.path.join(os.path.dirname(__file__), "rhsp-protocol.json")
    with open(out_path, "w") as fh:
        json.dump(doc, fh, indent=2)
        fh.write("\n")
    print(f"Wrote {out_path}: {len(doc['commands'])} commands, "
          f"{len(doc['responses'])} responses.")
