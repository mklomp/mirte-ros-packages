nano_map = {
    "RX0": 0,
    "TX1": 1,
    "D2": 2,
    "D3": 3,
    "D4": 4,
    "D5": 5,
    "D6": 6,
    "D7": 7,
    "D8": 8,
    "D9": 9,
    "D10": 10,
    "D11": 11,
    "D12": 12,
    "D13": 13,
    "A0": 14,
    "A1": 15,
    "A2": 16,
    "A3": 17,
    "A4": 18,
    "A5": 19,
    "A6": 20,
    "A7": 21,
}
max_pwm_value = 255
analog_offset = 14


def connectorToPins(connector):
    raise RuntimeError(f"Unknown conversion from connector {connector} to an IO number")


def pinNameToPinNumber(pin):
    if pin in nano_map:
        return nano_map[pin]
    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number")


mcu = "nano"
