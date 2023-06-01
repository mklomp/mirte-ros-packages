# No mapping, just assume the config is correct


def connectorToPins(connector):
    raise RuntimeError(f"Unknown conversion from connector {connector} to pins. Set a board type and version.")


def pinNameToPinNumber(pin):
    if int(pin) == pin:
        return pin
    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number")


max_pwm_value = 255
analog_offset = 0

mcu = "default"