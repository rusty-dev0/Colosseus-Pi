# warning: this file is AI-generated.
import serial
import re
import time

# ---------------- CONFIGURATION ----------------

UART_PORT = "/dev/serial0"
BAUD_RATE = 115200

# ------------------------------------------------

def connect():
    try:
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=BAUD_RATE,
            timeout=1
        )
        time.sleep(0.2)
        print(f"Connected to Pico on {UART_PORT}")
        return ser

    except serial.SerialException as e:
        print(f"Failed to connect to Pico: {e}")
        return None


def encode_command(speed=0, rotate=False, stop=False):
    """
    Command format: xxxrs101

    Bits 7-5: Speed (0-7)
    Bit 4:    Rotation
    Bit 3:    Stop
    Bits 2-0: Magic bits (101)
    """

    command = 0b101

    command |= (speed & 0b111) << 5

    if rotate:
        command |= 1 << 4

    if stop:
        command |= 1 << 3

    return command


def send_command(ser, command):
    ser.write(bytes([command]))
    ser.flush()

    print(f"Sent: {command:08b} (0x{command:02X})")


def process_command(ser, user_input):
    user_input = user_input.lower().strip()

    if user_input in ("stop", "halt"):
        command = encode_command(stop=True)
        send_command(ser, command)
        print("Robot stopping.")
        return

    if user_input in ("quit", "exit", "q"):
        return False

    # Match: "go speed 5" or "rotate speed 5"
    match = re.fullmatch(r"(go|rotate)\s+speed\s+(\d+)", user_input)

    if not match:
        print("Invalid command.")
        print("Examples: go speed 5, rotate speed 3, stop, quit")
        return

    movement = match.group(1)
    speed = int(match.group(2))

    if not 0 <= speed <= 7:
        print("Speed must be between 0 and 7.")
        return

    rotate = movement == "rotate"

    command = encode_command(
        speed=speed,
        rotate=rotate
    )

    send_command(ser, command)

    if rotate:
        print(f"Rotating at speed level {speed}.")
    else:
        print(f"Moving forward at speed level {speed}.")


def main():
    ser = connect()

    if ser is None:
        return

    print("\n===== ROBOT CONTROL =====")
    print("Commands:")
    print("  go speed 0-7")
    print("  rotate speed 0-7")
    print("  stop")
    print("  quit")
    print("=========================\n")

    try:
        while True:
            user_input = input("> ")

            result = process_command(ser, user_input)

            if result is False:
                break

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ser.close()
        print("UART connection closed.")


if __name__ == "__main__":
    main()
