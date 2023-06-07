import pico

mirte_pico_pcb_map06 = {
    "IR1": {"digital": "16", "analog": "26"},
    "IR2": {"digital": "17", "analog": "27"},
    "SRF1": {"trigger": "7", "echo": "6"},
    "SRF2": {"trigger": "9", "echo": "8"},
    "I2C1": {"scl": "5", "sda": "4"},
    "I2C2": {"scl": "11", "sda": "10"},
    "ENC1": {"pin": "15"},
    "ENC2": {"pin": "14"},
    "Keypad": {"pin": "28"},
    "Servo1": {
        "pin": "2"
    },  # These 2 servos don't work together with the motor controllers at the same time
    "Servo2": {
        "pin": "3"
    },  # These 2 servos don't work together with the motor controllers at the same time
    "Servo3": {"pin": "12"},
    "Servo4": {"pin": "13"},
    "LED": {"pin": "25"},
    "MC1-A": {"1a": "18", "1b": "19"},
    "MC1-B": {"1a": "20", "1b": "21"},
    "MC2-A": {"1a": "17", "1b": "27"},
    "MC2-B": {"1a": "16", "1b": "26"},
}


version = 0.6
mcu = "pico"
board_mapping = pico


def connectorToPins(
    connector,
):
    if version == 0.6:
        if connector in mirte_pico_pcb_map06:
            return mirte_pico_pcb_map06[connector]
    raise RuntimeError(
        f"Unknown conversion from connector {connector} to pins for Pico PCB v{version}."
    )


def pinNameToPinNumber(pin):
    return board_mapping.pinNameToPinNumber(pin)


def set_version(new_version):
    global version, mcu, board_mapping
    version = new_version
    if version == 0.6:
        mcu = "pico"
        board_mapping = pico
