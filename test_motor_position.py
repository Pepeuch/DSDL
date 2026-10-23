#!/usr/bin/env python3
"""Round-trip tests for uavcan.equipment.esc.MotorPosition (draft DTID 1037).

Run from the DSDL repository root: python3 test_motor_position.py
Requires: pip install dronecan
This test uses the DSDL definitions in the current checkout, not packaged ones.
"""
import os

import dronecan

ROOTS = [name for name in ('uavcan', 'com', 'ardupilot', 'dronecan')
         if os.path.isdir(name)]
assert 'uavcan' in ROOTS, 'Run from the DSDL repository root'
dronecan.load_dsdl(*ROOTS, exclude_dist=True)

Msg = dronecan.TYPENAMES['uavcan.equipment.esc.MotorPosition']
assert Msg.default_dtid == 1037
assert Msg.get_data_type_signature() == 0xB398385AA21637A6
assert Msg.get_min_bitlen() == 40
assert Msg.get_max_bitlen() == 56


def roundtrip(message, expected_frame_size):
    outgoing = dronecan.transport.Transfer(
        payload=message, source_node_id=42, transfer_id=3)
    frames = outgoing.to_frames()
    assert len(frames) == 1, 'Unexpected multi-frame transfer'
    frame = frames[0]
    assert len(frame.bytes) == expected_frame_size, frame.bytes
    assert len(frame.bytes) <= 8
    assert frame.start_of_transfer and frame.end_of_transfer
    assert (frame.message_id >> 8) & 0xFFFF == 1037

    incoming = dronecan.transport.Transfer()
    incoming.from_frames(frames)
    assert incoming.data_type_signature == 0xB398385AA21637A6
    return incoming.payload


checks = 0

# Tick representation: exact signed counts, including boundaries and reverse.
for count in (0, 1, -1, 123456, -123456, 2**31 - 1, -(2**31)):
    message = Msg()
    message.esc_index = 3
    message.valid = True
    message.origin_toggle = False
    message.position.ticks.ticks = count
    message.position.ticks.ticks_per_revolution = 42

    received = roundtrip(message, 8)  # 7-byte payload + 1-byte tail
    assert dronecan.get_active_union_field(received.position) == 'ticks'
    assert received.esc_index == 3
    assert received.valid is True
    assert received.origin_toggle is False
    assert received.position.ticks.ticks == count
    assert received.position.ticks.ticks_per_revolution == 42
    checks += 1

# Float representation: use exactly representable float32 values.
for revolutions in (0.0, 1.125, -12.5, 125000.0):
    message = Msg()
    message.esc_index = 31
    message.valid = True
    message.origin_toggle = True
    message.position.revolutions = revolutions

    received = roundtrip(message, 6)  # 5-byte payload + 1-byte tail
    assert dronecan.get_active_union_field(received.position) == 'revolutions'
    assert received.esc_index == 31
    assert received.valid is True
    assert received.origin_toggle is True
    assert received.position.revolutions == revolutions
    checks += 1

# Invalid telemetry must retain its invalid flag after transport.
message = Msg()
message.esc_index = 0
message.valid = False
message.origin_toggle = True
message.position.ticks.ticks = -42
message.position.ticks.ticks_per_revolution = 84
received = roundtrip(message, 8)
assert received.valid is False and received.origin_toggle is True
checks += 1

# Reference receiver arithmetic for wraparound (not a timing/continuity test).
def signed_wrap_delta(current, previous):
    return (current - previous + 2**31) % (2**32) - 2**31

assert signed_wrap_delta(-(2**31), 2**31 - 1) == 1
assert signed_wrap_delta(2**31 - 1, -(2**31)) == -1
checks += 2

print(f'OK: {checks} MotorPosition serialization and arithmetic checks')
print('DTID:', Msg.default_dtid)
print(f'Signature: 0x{Msg.get_data_type_signature():016X}')
print('Tick mode: 8-byte CAN frame | Float mode: 6-byte CAN frame')
