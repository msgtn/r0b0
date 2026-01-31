#!/usr/bin/env python3
"""
Reset the Pico and restart the blsm service from command line.

Usage:
    ./pico_reset_cli.py           # Reset Pico and restart service
    ./pico_reset_cli.py --no-service  # Only reset Pico, don't restart service
    ./pico_reset_cli.py --service-only  # Only restart service, don't reset Pico
"""

import argparse
import subprocess
import sys
import time

PICO_RUN_PIN = 24


def reset_pico():
    """Reset Pico by pulling RUN pin low momentarily"""
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("RPi.GPIO not available - skipping hardware reset")
        return False

    print("Resetting Pico...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PICO_RUN_PIN, GPIO.OUT, initial=GPIO.HIGH)
    
    GPIO.output(PICO_RUN_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(PICO_RUN_PIN, GPIO.HIGH)
    time.sleep(0.5)  # Wait for Pico to boot
    
    GPIO.cleanup(PICO_RUN_PIN)
    print("Pico reset complete")
    return True


def restart_service():
    """Restart the blsm service to reconnect serial"""
    print("Restarting blsm service...")
    result = subprocess.run(
        ["sudo", "systemctl", "restart", "blsm"],
        capture_output=True,
        text=True
    )
    if result.returncode == 0:
        print("Service restarted")
    else:
        print(f"Failed to restart service: {result.stderr}")
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Reset Pico and/or restart blsm service")
    parser.add_argument("--no-service", action="store_true", help="Only reset Pico, don't restart service")
    parser.add_argument("--service-only", action="store_true", help="Only restart service, don't reset Pico")
    args = parser.parse_args()

    if args.no_service and args.service_only:
        print("Error: --no-service and --service-only are mutually exclusive")
        sys.exit(1)

    if not args.service_only:
        reset_pico()

    if not args.no_service:
        restart_service()

    print("Done")


if __name__ == "__main__":
    main()
