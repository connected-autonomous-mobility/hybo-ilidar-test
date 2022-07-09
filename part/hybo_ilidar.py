"""
hybo_ilidar based on lidar.py
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

logger = logging.getLogger("donkeycar.parts.hybo_ilidar")

SERIAL_DEV = '/dev/ttyUSB0'


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
                new_scan  = raw_scan["points"]
                self.measurements.append(raw_scan)

                # test: show all points
                for point in new_scan:
                    print(point)
                                        
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

class HyboLidarPlot(object):
    '''
    based on LidarPlot2
    takes the lidar measurements as a list of (distance, angle) tuples
    and plots them to a PIL image which it outputs
    
    resolution: dimensions of image in pixels as tuple (width, height)
    plot_type: PLOT_TYPE_CIRC or PLOT_TYPE_LINE
    mark_px: size of data measurement marks in pixels
    max_dist: polar bounds; clip measures whose distance > max_dist
    angle_direction: direction of increasing angles in the data;
                     CLOCKWISE or COUNTER_CLOCKWISE
    rotate_plot: angle in positive degrees to rotate the measurement mark.
                 this can be used to match the direction of the robot
                 when it is plotted in world coordinates.
    '''
    PLOT_TYPE_LINE = 0
    PLOT_TYPE_CIRCLE = 1
    def __init__(self,
                 resolution=(500,500),
                 plot_type=PLOT_TYPE_CIRCLE,
                 mark_px=3,
                 max_dist=4000, #mm
                 rotate_plot=0,
                 background_color=(224, 224, 224),
                 border_color=(128, 128, 128),
                 point_color=(255, 64, 64)):
        
        self.frame = Image.new('RGB', resolution)
        self.mark_px = mark_px
        self.max_distance = max_dist
        self.resolution = resolution
        if plot_type == self.PLOT_TYPE_CIRCLE:
            self.mark_fn = mark_circle
        else:
            self.mark_fn = mark_line
        self.angle_direction = angle_direction
        self.rotate_plot = rotate_plot
        
        self.background_color = background_color
        self.border_color = border_color
        self.point_color = point_color

    def run(self, measurements):
        '''
        draw measurements to a PIL image and output the pil image
        measurements: list of cartesian coordinates as (x,y,z) tuples
        '''
            
        self.frame = Image.new('RGB', self.resolution, (255, 255, 255))
        bounds = (0, 0, self.frame.width, self.frame.height)
        draw = ImageDraw.Draw(self.frame)
        
        # background
        draw.rectangle(bounds, fill=self.background_color)

        # bounding perimeter and zero heading
        plot_polar_bounds(draw, bounds, self.border_color,
                          self.angle_direction, self.rotate_plot)
        plot_polar_angle(draw, bounds, self.border_color, 0,
                         self.angle_direction, self.rotate_plot)
        
        # data points
        plot_polar_points(
            draw, bounds, self.mark_fn, self.point_color, self.mark_px,
            [(distance, angle) for distance, angle, _, _, _ in measurements],
            self.max_distance, self.angle_direction, self.rotate_plot)
        
        return self.frame

    def shutdown(self):
        pass


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
