import sys
import glob
import serial


POLYNOMIAL = 0x1021
PRESET = 0

def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc


def crc16(data: bytes):
    '''
    CRC-16-CCITT Algorithm
    '''
    data = bytearray(data)
    crc = PRESET
    for b in data:
        cur_byte = 0xFF & b
        tmp = (crc >> 8) ^ cur_byte
        crc = (crc << 8) ^ _tab[tmp & 0xff]
        crc = crc & 0xffff
    
    return crc


_tab = [ _initial(i) for i in range(256) ]

print(hex(_tab[2]))

print(hex(crc16(b'\x31\x32\x33\x34\x35\x36\x37\x38\x39')))
