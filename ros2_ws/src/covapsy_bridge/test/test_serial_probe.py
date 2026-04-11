from covapsy_bridge.serial_probe import (
    build_baud_candidates,
    build_device_candidates,
    build_probe_pairs,
    split_csv_tokens,
)


def test_split_csv_tokens():
    assert split_csv_tokens('a, b,,c') == ['a', 'b', 'c']
    assert split_csv_tokens('') == []


def test_build_device_candidates_preserves_primary_and_uniques():
    devices = build_device_candidates(
        primary_device='/dev/stm32_mcu',
        candidates_csv='/dev/stm32_mcu,/dev/ttyACM1, /dev/ttyAMA0',
    )
    assert devices == ['/dev/stm32_mcu', '/dev/ttyACM1', '/dev/ttyAMA0']


def test_build_baud_candidates_dedup_and_filter_invalid():
    bauds = build_baud_candidates(
        primary_baud=115200,
        candidates_csv='57600, 115200, foo, -1, 230400',
    )
    assert bauds == [115200, 57600, 230400]


def test_build_probe_pairs_device_major_order():
    probes = build_probe_pairs(
        devices=['/dev/stm32_mcu', '/dev/ttyAMA0'],
        bauds=[115200, 57600],
    )
    assert probes == [
        ('/dev/stm32_mcu', 115200),
        ('/dev/stm32_mcu', 57600),
        ('/dev/ttyAMA0', 115200),
        ('/dev/ttyAMA0', 57600),
    ]

