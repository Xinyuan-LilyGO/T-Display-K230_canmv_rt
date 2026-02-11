import time, os, urandom, sys

from media.display import *
from media.media import *

#DISPLAY_WIDTH = ALIGN_UP(568, 16)
DISPLAY_WIDTH = 568
DISPLAY_HEIGHT = 1232

def display_test():
    print("display test")

    # create image for drawing
    img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)

    # use lcd as display output
    Display.init(Display.RM69A10, width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, to_ide = True)
    # init media manager
    MediaManager.init()

    try:
        while True:
            img.clear()
            for i in range(10):
                x = (urandom.getrandbits(11) % img.width())
                y = (urandom.getrandbits(11) % img.height())
                r = (urandom.getrandbits(8))
                g = (urandom.getrandbits(8))
                b = (urandom.getrandbits(8))
                size = (urandom.getrandbits(30) % 64) + 32
                # If the first argument is a scaler then this method expects
                # to see x, y, and text. Otherwise, it expects a (x,y,text) tuple.
                # Character and string rotation can be done at 0, 90, 180, 270, and etc. degrees.
                #img.draw_string_advanced(x,y,size, "Hello World!，你好世界！！！", color = (r, g, b),)
                # 绘制绿色矩形
            img.draw_rectangle(20, 20, 400, 400, color=(255, 0, 0), thickness=150)
            img.flood_fill(25, 25, color=(255, 0, 0), threshold=0, invert=False, clear_background=False)
            # draw result to screen
            Display.show_image(img)

            time.sleep(1)
            os.exitpoint()
    except KeyboardInterrupt as e:
        print("user stop: ", e)
    except BaseException as e:
        print(f"Exception {e}")

    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()

if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    display_test()
