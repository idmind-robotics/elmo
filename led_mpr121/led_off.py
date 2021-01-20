import spidev

spi = spidev.SpiDev()
spi.open(0,0)
spi.mode = 0b11
spi.max_speed_hz = 1000000
spi.bits_per_word = 8
spi.lsbfirst=False
start_frame = [0x00, 0x00, 0x00, 0x00]
end_frame = [0xFF, 0xFF, 0xFF, 0xFF]
off = start_frame + [0xE0, 0x00, 0x00, 0x00] *256 + end_frame
spi.writebytes(off)