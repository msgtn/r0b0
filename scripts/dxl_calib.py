import serial

serial = serial.Serial("/dev/ttyACM0", baudrate=115200)


def main():
    while True:
        angle = int(input("Motor angle: "))
        params: str = "&".join([f"{k}={angle}" for k in range(1, 4)])
        params += "\n"
        print(params)
        serial.write(bytes(params, encoding="utf-8"))


if __name__ == "__main__":
    main()
