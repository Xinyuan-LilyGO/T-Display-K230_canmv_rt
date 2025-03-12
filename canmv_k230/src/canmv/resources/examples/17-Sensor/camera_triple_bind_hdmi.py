# Camera Example
#
# Note: You will need an SD card to run this example.
#
# You can start 3 camera preview.
import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *

sensor0 = None
sensor1 = None
sensor2 = None

try:
    print("camera_test")

    # sensor0
    sensor0 = Sensor(id = 0, width = 1920, height = 1080)
    sensor0.reset()
    # set chn0 output size, 960x540
    sensor0.set_framesize(width = 960, height = 540)
    # set chn0 out format
    sensor0.set_pixformat(Sensor.YUV420SP)
    # bind sensor chn0 to display layer video 1
    bind_info = sensor0.bind_info(x = 0, y = 0)
    Display.bind_layer(**bind_info, layer = Display.LAYER_VIDEO1)

    # sensor1
    sensor1 = Sensor(id = 1, width = 1280, height = 960)
    sensor1.reset()
    # set chn0 output size
    sensor1.set_framesize(width = 960, height = 540)
    # set chn0 out format
    sensor1.set_pixformat(Sensor.YUV420SP)

    bind_info = sensor1.bind_info(x = 960, y = 0)
    Display.bind_layer(**bind_info, layer = Display.LAYER_VIDEO2)

    # sensor2
    sensor2 = Sensor(id = 2, width = 1280, height = 720)
    sensor2.reset()
    # set chn0 output size
    sensor2.set_framesize(width = 960, height = 540)
    # set chn0 out format
    sensor2.set_pixformat(Sensor.RGB888)

    # use hdmi as display output
    Display.init(Display.LT9611, to_ide = True)
    # init media manager
    MediaManager.init()

    # multiple sensor only need one excute run()
    sensor0.run()

    while True:
        os.exitpoint()
        img = sensor2.snapshot()
        Display.show_image(img, x = 0, y = 540)
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # multiple sensor all need excute stop()
    if sensor0 is not None:
        sensor0.stop()
    if sensor1 is not None:
        sensor1.stop()
    if sensor2 is not None:        
        sensor2.stop()
    # or call Sensor.deinit()
    # Sensor.deinit()

    # deinit display
    Display.deinit()

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # deinit media buffer
    MediaManager.deinit()
