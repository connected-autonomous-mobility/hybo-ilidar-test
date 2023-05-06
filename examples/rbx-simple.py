import time
import hybo

SERIAL_DEV = '/dev/ttyUSB0'

hybo = hybo.Lidar(SERIAL_DEV)
hybo.start()

# waiting for first frame
time.sleep(1)

while True:
    raw_scan = hybo.get_latest_frame()
    time.sleep(0.01)

    sequence  = raw_scan["sequence"]
    time_peak = raw_scan["time_peak"]
    new_scan  = raw_scan["points"]

    print(sequence, time_peak)
    for point in new_scan:
        print(point)

hybo.close()
