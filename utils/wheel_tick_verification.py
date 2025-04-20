import gpiod # sudo apt install gpiod python3-libgpiod -y, reboot after
import time

CHIP_NAME = "gpiochip0"  # default chip, find out using gpiodetect in terminal
LINE_OFFSET = 13         # GPIO 13 (BCM)
pulse_count = 0

chip = gpiod.Chip(CHIP_NAME)
line = chip.get_line(LINE_OFFSET)

# Request the line with both edge detection
line.request(consumer="encoder", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

print("Listening for encoder pulses on GPIO 13...")

try:
    while True:
        event = line.event_wait(sec=5)
        if event:
            ev = line.event_read()
            if ev.type == gpiod.LineEvent.RISING_EDGE:
                pulse_count += 1
                print(f"Pulse #{pulse_count}")
except KeyboardInterrupt:
    print("\nStopping...")
    line.release()
