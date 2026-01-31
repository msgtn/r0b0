#!/usr/bin/env python3
"""
Monitor GPIO pin 23 for button press. On press:
1. Reset the Pico by toggling the RUN pin (GPIO 24)
2. Restart the blsm service to reconnect serial

Wire the button between GPIO 23 and GND (uses internal pull-up).
Wire GPIO 24 to Pico's RUN pin to enable hardware reset.
"""

import subprocess
import time
import signal
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not available - install with: pip install RPi.GPIO")
    sys.exit(1)

BUTTON_PIN = 23  # Button input (active low with pull-up)
PICO_RUN_PIN = 24  # Connected to Pico's RUN pin for reset
DEBOUNCE_MS = 300


def reset_pico():
    """Reset Pico by pulling RUN pin low momentarily"""
    print("Resetting Pico...")
    GPIO.output(PICO_RUN_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(PICO_RUN_PIN, GPIO.HIGH)
    time.sleep(0.5)  # Wait for Pico to boot


def restart_service():
    """Restart the blsm service to reconnect serial"""
    print("Restarting blsm service...")
    subprocess.run(["sudo", "systemctl", "restart", "blsm"], check=False)


def on_button_press(channel):
    """Callback for button press"""
    print(f"Button pressed on GPIO {channel}")
    reset_pico()
    restart_service()
    print("Reset complete")


def cleanup(signum=None, frame=None):
    """Clean up GPIO on exit"""
    print("\nCleaning up...")
    GPIO.cleanup()
    sys.exit(0)


def main():
    # Set up signal handlers
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PICO_RUN_PIN, GPIO.OUT, initial=GPIO.HIGH)

    # Add interrupt for button press (falling edge = button pressed)
    GPIO.add_event_detect(
        BUTTON_PIN,
        GPIO.FALLING,
        callback=on_button_press,
        bouncetime=DEBOUNCE_MS
    )

    print(f"Listening for button press on GPIO {BUTTON_PIN}...")
    print(f"Pico reset pin: GPIO {PICO_RUN_PIN}")
    print("Press Ctrl+C to exit")

    # Keep running
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
