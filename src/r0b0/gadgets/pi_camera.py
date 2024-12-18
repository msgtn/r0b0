from .gadget import Gadget, Message
from .ST7789 import ST7789
from r0b0.config import TAPES_DIR
from r0b0.utils.loaders import decode_msg
from r0b0 import get_timestamp
import logging
# logging = logging.getLogger(__name__)
logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG,
    # level=getattr(logging,args.logging.upper())
)
import time
import os
import threading
from gpiozero import LED

from picamera2 import Picamera2
from libcamera import controls as libcamera_controls
import numpy as np
from time import sleep
from functools import partial
import glob
import subprocess
import libcamera
from libcamera import Transform
from PIL import Image

# Picamera2.set_logging(Picamera2.CRITICAL)
shutter_speeds = [1 / 30, 1 / 250, 1 / 1000]
shutter_speed_idx = 0
# RESOLUTION = (1920, 1080)
RESOLUTION = (2028, 1520)
RESOLUTION = (2400, 1800)
# RESOLUTION = (4624, 3472)
# RESOLUTION = (2828, 2120)
ROTATE = libcamera.Transform(hflip=1, vflip=1)
STILL_CONFIG_DICT = {
    "size": RESOLUTION,
}
FRAMERATE = 15
ISO = 800
SHUTTER_BLINK_SLEEP = 0.5
SENSOR_MODE = 3
FLASH = LED(12)

get_file_number = lambda save_dir: len(glob.glob(str(save_dir / "*")))

get_strftime = lambda : time.strftime('%y%m%d_%H%M%S')

subprocess.call(['raspi-gpio', 'set', '24', 'op'])
subprocess.call(['raspi-gpio', 'set', '24', 'dl'])

class PiCamera(Gadget, Picamera2):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # _PiCamera.__init__(self, sensor_mode=SENSOR_MODE)
        Picamera2.__init__(self)
        self.lcd_display = ST7789()
        self.lcd_display.Init()
        self.lcd_display.clear()
        self.lcd_display.bl_DutyCycle(0)

        ROTATE = libcamera.Transform(hflip=1, vflip=1)
        still_config = Picamera2.create_still_configuration(
                self, 
                *[STILL_CONFIG_DICT]*3,
                # transform=ROTATE, 
                transform=Transform(hflip=1, vflip=1), 
                # buffer_count=4,
                queue=False
                )
        self.still_configuration.size = RESOLUTION
        #self.still_configuration.size = (4056,3040)
        # self.still_configuration.size = (2028, 1520)
        # self.still_configuration.transform = ROTATE
        Picamera2.configure(self, still_config)
        # breakpoint()
        Picamera2.start(self, "still", show_preview=False)
        self.set_controls({
            "AfMode": libcamera_controls.AfModeEnum.Manual,
            "AeEnable": False,
            "AeFlickerMode": libcamera_controls.AeFlickerModeEnum.Off,
            "ExposureTime":int(1e6/250),
            # "AnalogueGain":1.0,
            "AnalogueGain":4.0,
            })
        ROTATE = libcamera.Transform(hflip=1, vflip=1)
        # self.set_logging(self.CRITICAL)

        # breakpoint()  
        try:
            os.remove(str(TAPES_DIR / "testshot.jpg"))
        except:
            pass
        self.capture_file(str(TAPES_DIR / "testshot.jpg"))

        self.on("shutter", handler=self.release_shutter, namespace=self.namespace)
        try:
            logging.debug(self.camera_controls)
        except:
            pass

        self.on("d_down", handler=self.shutter15, namespace=self.namespace)
        self.on("d_right", handler=self.shutter60, namespace=self.namespace)
        self.on("d_up", handler=self.shutter250, namespace=self.namespace)
        self.on("d_left", handler=self.shutter1000, namespace=self.namespace)
        self.on("k1", handler=self.on_show_image, namespace=self.namespace)
        self.on("k2", handler=self.on_show_image, namespace=self.namespace)
        self.on("k3", handler=self.on_show_image, namespace=self.namespace)

        self.on(
            "set_shutter_speed",
            handler=self.on_set_shutter_speed,
            namespace=self.namespace,
        )
        self.set_param = self.__dict__.update
        self.start = lambda: Gadget.start(self)
        self.t_last = time.time()
        self.exposing = False

    @decode_msg
    def release_shutter(self, msg, save_dir=TAPES_DIR, **kwargs):
        if self.exposing or time.time()-self.t_last < 0.5:
            return
        subprocess.call(['raspi-gpio', 'set', '47', 'dh'])
        logging.debug(f"Shutter released")
        # TODO - split off into separate subfolders
        # instead of one large folder
        capture_thread = threading.Thread(
                target=self._capture_file,)
        flash_thread = threading.Thread(target=self.trigger_flash)
        # capture_thread.start()
        flash_thread.start()
        self._capture_file(save_dir, **kwargs)

    def trigger_flash(self, period=1/6):
        num_files_0 = get_file_number(TAPES_DIR)
        while not self.exposing:
            continue
        # while self.exposing:
        t_start = time.time()
        # while get_file_number(TAPES_DIR)==num_files_0 or len(self._requests)==0 or self.exposing:
        while self.exposing:
        # while not self.lock.locked():
            t = time.time()
            if t-t_start > period:
                logging.info("Triggering flash")
                FLASH.on()
                FLASH.off()
                t_start = t


    def _capture_file(self, save_dir=TAPES_DIR, **kwargs):
        self.t_last = time.time()
        logging.info("Taking picture")
        # filename = str(TAPES_DIR / f"picam_{get_file_number(TAPES_DIR)}.jpg")
        filename = str(TAPES_DIR / f"picam_{get_strftime()}.jpg")
        t_start = time.time()
        self.exposing = True
        self.capture_file(filename, wait=True)
        # image = self.capture_image("main", wait=True)

        # image = Image.open(filename)
        # image = image.transpose(Image.ROTATE_180)
        # image.save(filename, exif=image.info["exif"])
        self.exposing = False
        t_end = time.time()
        del_t = t_end - t_start
        logging.debug(f"Time to capture: {del_t:0.2f}")
        # self.capture_file(filename)

    @decode_msg
    def set_shutter(self, shutter_speed):
        """
        Set the shutter speed of the camera.

        :param shutter_speed: Shutter speed in milliseconds, i.e. 1e6 / {15, 60, 250}.
        """
        logging.debug(f"Shutter speed: 1/{int(1e6 / shutter_speed)}")
        self.set_controls({"ExposureTime": shutter_speed})

    @decode_msg
    def shutter15(
        self,
        msg,
    ):
        logging.info("Shutter: 1/2")
        # self.set_controls({"ExposureTime": int(1e6 / 1)})
        self.set_controls({"ExposureTime": int(1e6 / 2)})
        return

    @decode_msg
    def shutter60(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/60")
        self.set_controls({"ExposureTime": int(1e6 / 15)})
        return

    @decode_msg
    def shutter250(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/250")
        self.set_controls({"ExposureTime": int(1e6 / 60)})
        return

    @decode_msg
    def shutter1000(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/1000")
        self.set_controls({"ExposureTime": int(1e6 / 250)})
        return

    @decode_msg
    # def on_set_shutter_speed(self, shutter_speed: float, **kwargs):
    def on_set_shutter_speed(self, data, **kwargs):
        msg = data["msg"]
        ss = int(msg.shutter_speed)
        logging.debug(f"Shutter: {ss}")
        self.set_controls({"ExposureTime": ss})

    @decode_msg
    def on_show_image(self, data):
        filename = str(TAPES_DIR / "picam_2412*")
        filename = sorted(glob.glob(filename))[-1]
        # filename = str(TAPES_DIR / "picam_9999.jpg")
        filename = os.path.abspath(filename)

        
        subprocess.call(['raspi-gpio', 'set', '24', 'dh'])
        logging.warning(filename)
        print(filename)
        image = Image.open(filename)
        image = image.resize((240,240))
        self.lcd_display.clear()
        self.lcd_display.bl_DutyCycle(50)

        self.lcd_display.ShowImage(image)
        time.sleep(2)
        self.lcd_display.clear()
        self.lcd_display.bl_DutyCycle(0)

        subprocess.call(['raspi-gpio', 'set', '24', 'dl'])
