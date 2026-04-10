from covapsy_bridge.usb_protocol import (
    encode_command_line,
    encode_lcd_line,
    parse_telemetry_line,
    parse_wire_bool,
    sanitize_lcd_cell,
)


def test_encode_command_line_shape():
    line = encode_command_line(
        seq=17,
        steer_deg=12.3456,
        speed_mps=0.789,
        run_enable=True,
        emergency_brake=False,
    )
    assert line == 'CMD,17,12.346,0.789,1,0\n'


def test_parse_telemetry_line_valid():
    seq, wheel, rear, status = parse_telemetry_line('TEL,42,0.55,1,3')
    assert seq == 42
    assert wheel == 0.55
    assert rear is True
    assert status == 3


def test_parse_wire_bool_accepts_text():
    assert parse_wire_bool('true') is True
    assert parse_wire_bool('False') is False


def test_parse_telemetry_line_rejects_invalid_prefix():
    try:
        parse_telemetry_line('BAD,1,0.2,0,0')
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_parse_telemetry_line_rejects_invalid_bool():
    try:
        parse_telemetry_line('TEL,1,0.2,maybe,0')
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_parse_telemetry_line_rejects_non_finite_speed():
    try:
        parse_telemetry_line('TEL,1,nan,1,0')
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_sanitize_lcd_cell_and_encode_lcd_line():
    assert sanitize_lcd_cell('abC,|z') == 'ABC//Z'
    assert sanitize_lcd_cell('12345678901234567890') == '1234567890123456'

    line = encode_lcd_line(
        seq=9,
        lines=['Mode race', 'usb,ok', 'bad|char', '12345678901234567890'],
    )
    assert line == 'LCD,9,MODE RACE|USB/OK|BAD/CHAR|1234567890123456\n'


def test_encode_lcd_line_requires_four_lines():
    try:
        encode_lcd_line(seq=1, lines=['a', 'b', 'c'])
        assert False, 'expected ValueError'
    except ValueError:
        pass
