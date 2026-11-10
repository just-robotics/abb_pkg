# /jointN/device_state: state of a CANopen drive, 1 Hz.

uint8 DISCONNECTED = 0
uint8 OK = 1
uint8 ERROR = 2
uint8 WARNING = 3
uint8 UNKNOWN = 4

uint8 state     # one of the values above
string error    # error text of the drive
