from rpi_ws281x import PixelStrip, Color


class LEDStrip:
    def __init__(self, count, pin=18, freq=800000, dma=10, brightness=255, invert=False, channel=0):
        self.count = count
        self.strip = PixelStrip(count, pin, freq, dma, invert, brightness, channel)
        self.strip.begin()

    @property
    def length(self):
        return self.count

    def update(self):
        self.strip.show()

    def set_color(self, i, color, update=True):
        self.strip.setPixelColor(i, color)
        if update:
            self.update()
