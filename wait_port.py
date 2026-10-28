# The Teensy reboots after an upload and is off USB for a second or two.
# Without this, "Upload and Monitor" starts the monitor in that gap and it
# finds no /dev/cu.usbmodem* port on macOS. Wait for the port to go down and come back.
Import("env")
import glob, sys, time

def wait_for_teensy(source, target, env):
    if sys.platform != "darwin":
        return
    port = "/dev/cu.usbmodem*"
    t0 = time.time()
    while glob.glob(port) and time.time() - t0 < 2:       # going down for the reboot
        time.sleep(0.05)
    while not glob.glob(port) and time.time() - t0 < 15:  # and back up
        time.sleep(0.05)
    time.sleep(0.5)                                        # let it settle

env.AddPostAction("upload", wait_for_teensy)
