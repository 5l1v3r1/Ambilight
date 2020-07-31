import serial
from time import sleep
import thread

SERIAL_DEVICE = '/dev/ttyACM0'
SERIAL_SPEED = 115200
DELIMETTER = ord('\n') # tof!
DELIMETTER_FIXER = 254

class LEDStrip:
    def __init__(self, count):
        self._serial = serial.Serial(SERIAL_DEVICE, SERIAL_SPEED)
        self._strip = [[0, 0, 0] for _ in range(count)]
        self._fps = 10
        thread.start_new_thread(self._update)

    @property
    def length(self):
        return len(self._strip)

    def _update():
        speed = 1 # FIX BUG, slow start arduino to not stuck
        while True:
            speed = speed * 2 if speed < self._fps else self._fps
            sleep(1 / speed)
            flatted_strip = sum(self._strip, [])
            character_strip = map(lambda x: chr(x) if x != 255 else chr(DELIMETTER_FIXER), flatted_strip)
            self.serial.write(''.join(character_strip)) + DELIMETTER)

    def set_color(self, i, color):
        self.strip[i] = color
