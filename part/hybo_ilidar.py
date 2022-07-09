"""
Hybo based on lidar.py
"""
#
# requies glob to be installed: "pip3 install glob2"
# requires Adafruit RPLidar driver to be installed:
#   pip install Adafruit_CircuitPython_RPLIDAR
#
import logging
import sys
import time
import math
import pickle
from turtle import distance
import serial
import numpy as np
from donkeycar.utils import norm_deg, dist, deg2rad, arr_to_img
from PIL import Image, ImageDraw

import time
import hybo
#from hybo import Lidar

logger = logging.getLogger("donkeycar.parts.hybo")

CLOCKWISE = 1
COUNTER_CLOCKWISE = -1
SERIAL_DEV = '/dev/ttyUSB0'


def limit_angle(angle):
    """
    make sure angle is 0 <= angle <= 360
    """
    while angle < 0:
        angle += 360
    while angle > 360:
        angle -= 360
    return angle


def angle_in_bounds(angle, min_angle, max_angle):
    """
    Determine if an angle is between two other angles.
    """
    if min_angle <= max_angle:
        return min_angle <= angle <= max_angle
    else:
        # If min_angle < max_angle then range crosses
        # zero degrees, so break up test
        # into two ranges
        return (min_angle <= angle <= 360) or (max_angle >= angle >= 0)

class HyboLidar(object):
    '''
    Adapted RP2Lidar
    '''
    def __init__(self,
                 batch_ms=5000,  # how long to loop in run()
                 debug=False):
        
        self.lidar = None
        self.port = None
        self.on = False

        self.measurements = [] # list of (distance, angle, time, scan, index) 

        #from adafruit_rplidar import RPLidar
        import glob
        
        #
        # find the serial port where the lidar is connected
        #
        port_found = False
        temp_list = glob.glob ('/dev/ttyUSB*')
        result = []
        for a_port in temp_list:
            try:
                s = serial.Serial(a_port)
                s.close()
                result.append(a_port)
                port_found = True
            except serial.SerialException:
                pass
        if not port_found:
            raise RuntimeError("No Hybo iLidar is connected.")

        # initialize
        self.port = result[0]
        print(f"Found & Connecting to {self.port}")
        self.hybo = hybo.Lidar(self.port)
        self.hybo.start()
        time.sleep(1)

        self.measurement_count = 0  # number of measurements in the scan
        self.measurement_index = 0  # index of next measurement in the scan
        self.full_scan_count = 0
        self.full_scan_index = 0
        self.total_measurements = 0
        #self.iter_measurements = self.lidar.iter_measurements()
        self.measurement_batch_ms = batch_ms
        self.measurements = []

        self.running = True

    def poll(self):
        if self.running:
            try:
                #
                # read one measurement
                #
                #new_scan, quality, angle, distance = next(self.iter_measurements)  # noqa
                raw_scan = self.hybo.get_latest_frame()  # noqa
                print(raw_scan)
                #sequence  = raw_scan["sequence"]
                #time_peak = raw_scan["time_peak"]
                #distance  = raw_scan["points"]
                #new_scan  = raw_scan["points"]
                self.measurements.append(raw_scan)
                                        
                now = time.time()
                self.total_measurements += 1

                # check for start of new scan
                if raw_scan:
                    self.full_scan_count += 1
                    self.full_scan_index = 0
                    self.measurement_count = self.measurement_index  # this full scan
                    self.measurement_index = 0   # start filling in next scan
                             
            except serial.serialutil.SerialException:
                logger.error('SerialException from Hybo iLidar.')

    def update(self):
        start_time = time.time()
        while self.running:
            self.poll()
            time.sleep(0)  # yield time to other threads
        total_time = time.time() - start_time
        scan_rate = self.full_scan_count / total_time
        measurement_rate = self.total_measurements / total_time
        logger.info("HyboLidar total scan timfrom hybo import Lidar e = {time} seconds".format(time=total_time))
        logger.info("HyboLidar total scan count = {count} scans".format(count=self.full_scan_count))
        logger.info("HyboLidar total measurement count = {count} measurements".format(count=self.total_measurements))
        logger.info("HyboLidar rate = {rate} scans per second".format(rate=scan_rate))
        logger.info("HyboLidar rate = {rate} measurements per second".format(rate=measurement_rate))

    def run_threaded(self):
        if self.running:
            return self.measurements
        return []
    
    def run(self):
        if not self.running:
            return []
        #
        # poll for 'batch' and return it
        # poll for time provided in constructor
        #
        batch_time = time.time() + self.measurement_batch_ms / 1000.0
        while True:
            self.poll()
            time.sleep(0)  # yield time to other threads
            if time.time() >= batch_time:
                break
        return self.measurements

    def shutdown(self):
        self.running = False
        time.sleep(2)
        if self.hybo is not None:
            self.hybo.close()
            self.hybo = None


if __name__ == "__main__":
    import argparse
    import cv2
    import json
    from threading import Thread

    import time
    import hybo
    
    def convert_from_image_to_cv2(img: Image) -> np.ndarray:
        # return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        return np.asarray(img)
    
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--rate", type=float, default=20,
                        help = "Number of scans per second")
    parser.add_argument("-n", "--number", type=int, default=40,
                        help = "Number of scans to collect")
    parser.add_argument("-a", "--min-angle", type=float, default=0,
                        help="Minimum angle in degress (inclusive) to save")
    parser.add_argument("-A", "--max-angle", type=float, default=360,
                        help="Maximum angle in degrees (inclusive) to save")
    parser.add_argument("-d", "--min-distance", type=float, default=sys.float_info.min,  # noqa
                        help="Minimum distance (inclusive) to save")
    parser.add_argument("-D", "--max-distance", type=float, default=4000,
                        help="Maximum distance (inclusive) to save")
    parser.add_argument("-f", "--forward-angle", type=float, default=0.0,
                        help="Forward angle - the angle facing 'forward'")
    parser.add_argument("-s", "--angle-direction", type=int, default=COUNTER_CLOCKWISE,  # noqa
                        help="direction of increasing angles (1 is clockwise, -1 is counter-clockwise)")  # noqa
    parser.add_argument("-p", "--rotate-plot", type=float, default=0.0,
                        help="Angle in degrees to rotate plot on cartesian plane")  # noqa
    parser.add_argument("-t", "--threaded", action='store_true', help = "run in threaded mode")

    # Read arguments from command line
    args = parser.parse_args()
    
    help = []
    if args.rate < 1:
        help.append("-r/--rate: must be >= 1.")
        
    if args.number < 1:
        help.append("-n/--number: must be >= 1.")
        
    if args.min_distance < 0:
        help.append("-d/--min-distance must be >= 0")

    if args.max_distance <= 0:
        help.append("-D/--max-distance must be > 0")
        
    if args.min_angle < 0 or args.min_angle > 360:
        help.append("-a/--min-angle must be 0 <= min-angle <= 360")

    if args.max_angle <= 0 or args.max_angle > 360:
        help.append("-A/--max-angle must be 0 < max-angle <= 360")
      
    if args.forward_angle < 0 or args.forward_angle > 360:
        help.append("-f/--forward-angle must be 0 <= forward-angle <= 360")
        
    if args.angle_direction != CLOCKWISE and \
       args.angle_direction != COUNTER_CLOCKWISE:
        help.append("-s/--angle-direction must be 1 (clockwise) or -1 (counter-clockwise)")  # noqa
        
    if args.rotate_plot < 0 or args.rotate_plot > 360:
        help.append("-p/--rotate-plot must be 0 <= min-angle <= 360")
        
    if len(help) > 0:
        parser.print_help()
        for h in help:
            print("  " + h)
        sys.exit(1)
        
    lidar_thread = None
    lidar = None
    
    try:
        scan_count = 0
        seconds_per_scan = 1.0 / args.rate
        scan_time = time.time() + seconds_per_scan

        #
        # construct a lidar part
        #
        print(f"main: Connecting to lidar...")
        lidar = HyboLidar(batch_ms=1000.0/args.rate)
        print(f"main: Connected to lidar.")
        
        #
        # construct a lidar plotter
        #
        """
        plotter = LidarPlot2(plot_type=LidarPlot2.PLOT_TYPE_CIRCLE,
                             max_dist=args.max_distance,
                             angle_direction=args.angle_direction,
                             rotate_plot=args.rotate_plot,
                             background_color=(32, 32, 32),
                             border_color=(128, 128, 128),
                             point_color=(64, 255, 64))        
        #
        # start the threaded part
        # and a threaded window to show plot
        #
        cv2.namedWindow("lidar")
        if args.threaded:
            lidar_thread = Thread(target=lidar.update, args=())
            lidar_thread.start()
            cv2.startWindowThread()
        """
        
        while scan_count < args.number:
            start_time = time.time()

            # emit the scan
            scan_count += 1
            print(f"main: emitting scan {scan_count}")

            # get most recent scan and plot it
            if args.threaded:
                measurements = lidar.run_threaded()
            else:
                measurements = lidar.run()
            
            """
            img = plotter.run(measurements)
            
            # show the image in the window
            cv2img = convert_from_image_to_cv2(img)
            cv2.imshow("lidar", cv2img)
            """
            
            if not args.threaded:
                key = cv2.waitKey(1) & 0xFF
                if 27 == key or key == ord('q') or key == ord('Q'):
                    break

            # yield time to background threads
            sleep_time = seconds_per_scan - (time.time() - start_time)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                time.sleep(0)  # yield time to other threads

    except KeyboardInterrupt:
        print('Stopping early.')
    except Exception as e:
        print(e)
        exit(1)
    finally:
        if lidar is not None:
            lidar.shutdown()
            #plotter.shutdown()
            cv2.destroyAllWindows()
        if lidar_thread is not None:
            lidar_thread.join()  # wait for thread to end
