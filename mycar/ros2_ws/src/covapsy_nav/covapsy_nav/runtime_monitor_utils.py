"""Pure utilities used by runtime monitor logging."""


def parse_status_blob(raw: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in raw.split(";"):
        token = token.strip()
        if not token:
            continue
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key.strip()] = value.strip()
        elif "status" not in fields:
            fields["status"] = token
    return fields
