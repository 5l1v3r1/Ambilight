import serial
from time import sleep
from threading import Thread

SERIAL_DEVICE = '/dev/ttyACM0'
SERIAL_SPEED = 115200
DELIMETTER = ord('\n')
DELIMETTER_FIXER = ord('\n') + 1 

class LEDStrip:
    def __init__(self, count):
        self._serial = serial.Serial(SERIAL_DEVICE, SERIAL_SPEED)
        self._strip = [[0, 0, 0] for _ in range(count)]
        self._fps = 100
        Thread(target=self._update).start()

    @property
    def length(self):
        return len(self._strip)

    def _update(self):
        speed = 1 # FIX BUG, slow start arduino to not stuck
        while True:
#            print(self._strip)
            speed = speed * 1.1 if speed < self._fps else self._fps
            sleep(1 / speed)
            flatted_strip = sum(self._strip, [])
            filtred_strip = [i if i != DELIMETTER else DELIMETTER_FIXER for i in flatted_strip]
            self._serial.write(bytearray(filtred_strip) + b'\n')

    def set_color(self, i, color):
        # BRG
        self._strip[i] = [color[0], color[2], color[1]]


import numpy as np
import cv2


class ColorPicker:
    def __init__(self):
        self.corners = [(87, 132), (517, 107), (62, 370), (537, 372)]
        self.target_height = 900
        self.target_width = 1600

        grids = self._get_grids((self.target_width, self.target_height), 13, 24, 20)
        border_points = self._get_border_points(grids)
        self.strip_points = self._map_border_order_points(border_points)

    def _get_flat_tv(self, image):
        origin = np.float32(self.corners)
        target = np.float32([
            [0, 0],
            [self.target_height, 0],
            [0, self.target_width],
            [self.target_height,
             self.target_width]
        ])

        M = cv2.getPerspectiveTransform(origin, target)
        return cv2.warpPerspective(image, M, (self.target_height, self.target_width))

    def _get_grids(self, tv_size, width_count, height_count, depth):
        width = tv_size[0] - 2 * depth
        height = tv_size[1] - 2 * depth
        width_size = width // width_count
        height_size = height // height_count

        left = (width - (width_size * width_count)) // 2 + depth
        top = (height - (height_size * height_count)) // 2 + depth

        result = []
        for x in range(width_count):
            row = []
            for y in range(height_count):
                point_x = int(left + y * height_size + .5 * height_size)
                point_y = int(top + x * width_size + .5 * width_size)
                row.append((point_x, point_y))
            result.append(row)
        return result

    def _get_border_points(self, grid_points):
        border_points = []
        for row in reversed(grid_points):
            border_points.append(row[0])
        border_points += grid_points[0][1:len(grid_points[0]) - 1]
        for row in grid_points:
            border_points.append(row[len(row) - 1])

        border_points += reversed(grid_points[len(grid_points) - 1][1:len(grid_points[0]) - 1])
        return border_points

    def _find_first_point(self, border_points):
        return 66

    def _find_last_point(self, border_points):
        return 52

    def _map_border_order_points(self, border_points):
        first = self._find_first_point(border_points)
        last = self._find_last_point(border_points)

        border_points = border_points[first:] + border_points[:first]
        return border_points[:len(border_points) - first + last]

    def _find_main_color_of_point(self, image, point):
        return image[point[1], point[0]]

    def get_strip_colors(self, image):
        tv_image = self._get_flat_tv(image)
        return [self._find_main_color_of_point(tv_image, point) for point in self.strip_points]



import numpy as np
import cv2
from rpi_ws281x import PixelStrip, Color
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

if __name__ == '__main__':
    color_picker = ColorPicker()
    leds = LEDStrip(58)

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        
        for i, color in enumerate(color_picker.get_strip_colors(image)):
            leds.set_color(i, color)

        # cv2.imshow("Frame", image)
        # key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        # if key == ord("q"):
        #     break

    cv2.destroyAllWindows()
