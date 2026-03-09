from covapsy_nav.runtime_monitor_utils import parse_status_blob


def test_parse_status_blob_extracts_key_value_fields():
    raw = "backend=spi;ok:spi:bus0.dev1@1000000;race_running=1;stop_latched=0"
    fields = parse_status_blob(raw)
    assert fields["backend"] == "spi"
    assert fields["race_running"] == "1"
    assert fields["stop_latched"] == "0"
    assert fields["status"] == "ok:spi:bus0.dev1@1000000"


def test_parse_status_blob_ignores_empty_tokens():
    fields = parse_status_blob(";;backend=uart;; ;race_running=0;")
    assert fields == {"backend": "uart", "race_running": "0"}
