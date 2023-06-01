# Map to convert from Mirte PCB to STM32 pins numbers
# This should be the same as printed on the PCB
mirte_pcb_map = {
 "IR1"   : {"digital": "C15", "analog": "A0" },
 "IR2"   : {"digital": "B0" , "analog": "A1" },
 "SRF1"  : {"trigger": "A15", "echo"  : "C14"},
 "SRF2"  : {"trigger": "A5" , "echo"  : "A6" },
 "I2C1"  : {"scl"    : "B6" , "sda"   : "B7" },
 "I2C2"  : {"scl"    : "B10", "sda"   : "B11"},
 "ENCA"  : {"pin"    : "B4" },
 "ENCB"  : {"pin"    : "B12"},
 "Keypad": {"pin"    : "A4" },
 "Servo1": {"pin"    : "B5" },
 "Servo2": {"pin"    : "A7" },
 "LED"   : {"pin"    : "C13"},
 "MA"    : {"1a"     : "A8" , "1b"    : "B3" },
 "MB"    : {"1a"     : "B14", "1b"    : "B15"},
 "MC"    : {"1a"     : "B1" , "1b"    : "A10"},
 "MD"    : {"1a"     : "A9" , "1b"    : "B13"}
}


# Map to convert from STM32 to pin numbers
# TODO: maybe convert this into JSON files and load it from there
# TODO: maybe just make this an list and get the index of the list
# And/or put this in the param server so teh web interface can also use this
stm32_map = {
 "B9" : 0,
 "B8" : 1,
 "B7" : 2,
 "B6" : 3,
 "B5" : 4,
 "B4" : 5,
 "B3" : 6,
 "A15": 7,
 "A12": 8,
 "A11": 9,
 "A10": 10,
 "A9" : 11,
 "A8" : 12,
 "B15": 13,
 "B14": 14,
 "B13": 15,
 "B12": 16,
 "C13": 17,    # LED
 "C14": 18,
 "C15": 19,
 "A0" : 20,
 "A1" : 21,
 "A2" : 22,
 "A3" : 23,
 "A4" : 24,
 "A5" : 25,
 "A6" : 26,
 "A7" : 27,
 "B0" : 28,
 "B1" : 29,
 "B10": 30,
 "B11": 31
}
# I thought this should be 65535 for the SEM, but for
# some reason we need 255
max_pwm_value = 255
analog_offset = 20



def connectorToPins(connector ):
    return mirte_pcb_map[connector]


def pinNameToPinNumber(pin):
    if pin in stm32_map:
        return stm32_map[pin]
    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number")
