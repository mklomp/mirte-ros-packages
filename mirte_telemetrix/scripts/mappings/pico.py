# Map pcb names to pins, based on version number

def pinNameToPinNumber(pin):  # pico has no mapping
    if pin.isdigit():
        return int(pin)

    raise RuntimeError(
        f"Unknown conversion from pin {pin} to an IO number, {int(pin)} == { pin}"
    )


analog_offset = 26
max_pwm_value = 255

mcu = "pico"

i2c_port0_sda_pins = [0, 4, 8, 12, 20, 16]
i2c_port1_sda_pins = [2, 6, 10, 14, 26, 18]


def getI2CPort(sda):
    if sda in i2c_port0_sda_pins:
        return 0
    elif sda in i2c_port1_sda_pins:
        return 1
    else:
        raise RuntimeError(f"No i2c port for SDA pin {sda}.")
