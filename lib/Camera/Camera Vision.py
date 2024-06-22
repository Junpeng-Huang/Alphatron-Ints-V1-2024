import sensor, image, time
import pyb
import math
from pyb import UART

YELLOW = 1
BLUE = 2
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
draw = True

# Individual
thresholds = [(54, 71, -18, 1, 25, 51),(39, 50, -16, -3, -36, -1)] # Yellow First
ballthreshold = [(42, 72, 9, 64, 27, 59)] #Ball

# Superteam
#if ROBOT_A:
    #thresholds = [((39, 57, 5, 40, 8, 57),), ((17, 26, -2, 41, -59, -14),)] # Yellow  is first
#else:
    #thresholds = [((38, 61, 4, 43, 7, 64),), ((16, 21, -1, 23, -35, -16),)] # Yellow  is first

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QsVGA (320x240)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)#False, (-1.72093, -6.02073, -3.45436))
sensor.set_brightness(1)
sensor.set_contrast(0)
#sensor.set_saturation(0)
#sensor.set_auto_exposure(False, exposure_us=150000)
clock = time.clock()
uart = UART(3, 9600, timeout_char = 10)

while(True):
    data = [160, 120, 160, 120, 160, 120]
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(thresholds, x_stride=2, y_stride = 2, area_threshold = 50, pixel_threshold = 200, merge = False, margin = 23)
    blobs = sorted(blobs, key=lambda blob: -blob.area())
    orange = None
    blue = None
    for blob in blobs:
        if math.sqrt((blob.cx()-160)**2 + (blob.cy()-120)**2) < 100:
            if data[0] == 160 and data[1] == 120 and blob.code() == YELLOW:
                data[0] = 320 - blob.cx()
                data[1] = 240 - blob.cy()
            if data[2] == 160 and data[3] == 120 and blob.code() == BLUE:
                data[2] = 320 - blob.cx()
                data[3] = 240 - blob.cy()

    ballblobs = img.find_blobs(ballthreshold, x_stride=2, y_stride = 2, area_threshold = 10, pixel_threshold = 200, merge = False, margin = 23)
    ballblobs = sorted(ballblobs, key=lambda blob: -blob.area())
    yellow = None
    for blob in ballblobs:
        if math.sqrt((blob.cx()-160)**2 + (blob.cy()-120)**2) < 100:
            if data[4] == 160 and data[5] == 120:
                data[4] = 320 - blob.cx()
                data[5] = 240 - blob.cy()

    uart.writechar(255)
    uart.writechar(255)
    for byte in data:
        uart.writechar(byte)
        if draw:
            img.draw_circle(160, 120, 100)
            img.draw_line(int(round(FRAME_WIDTH)/2 - 10), int(round(FRAME_HEIGHT / 2)), int(round(FRAME_WIDTH / 2) + 10), int(round(FRAME_HEIGHT / 2)))
            img.draw_line(int(round(FRAME_WIDTH)/2), int(round(FRAME_HEIGHT / 2) + 10), int(round(FRAME_WIDTH / 2)), int(round(FRAME_HEIGHT / 2) -10))
            img.draw_line(int(round(FRAME_WIDTH)/2), int(round(FRAME_HEIGHT / 2) + 10), int(round(FRAME_WIDTH / 2)), int(round(FRAME_HEIGHT / 2) -10))
            img.draw_line(round(FRAME_WIDTH / 2), round(FRAME_HEIGHT / 2), 320 - data[4], 240 - data[5])
            img.draw_line(round(FRAME_WIDTH / 2), round(FRAME_HEIGHT / 2), 320 - data[2], 240 - data[3])
            img.draw_line(round(FRAME_WIDTH / 2), round(FRAME_HEIGHT / 2), 320 - data[0], 240 - data[1])
