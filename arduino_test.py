import serial
import time


def test_servos():
    try:
        # Connect to Arduino
        arduino = serial.Serial(
            port='/dev/cu.usbmodem1401', baudrate=9600, timeout=1)
        print("Connected to Arduino on /dev/cu.usbmodem1401")
        time.sleep(2)  # Allow time for Arduino to reset

        # Test angles for each servo (min, middle, max)
        test_angles = [145]

        # For each servo (0-4)
        for servo_idx in range(5):
            print(f"\n--- Testing Servo {servo_idx} ---")

            # Test each angle
            for angle in test_angles:
                # Create command string (e.g., "S0,90")
                command = f"S{servo_idx},{angle}\n"
                print(f"Sending: {command.strip()}")

                # Send to Arduino
                arduino.write(command.encode('utf-8'))

                # Read and print response
                time.sleep(0.5)
                while arduino.in_waiting:
                    response = arduino.readline().decode('utf-8').strip()
                    print(f"Arduino: {response}")

                # Wait before next command
                time.sleep(1)

        # Reset all servos to neutral position
        print("\n--- Resetting all servos to neutral position ---")
        arduino.write(b"ALL\n")
        time.sleep(0.5)
        while arduino.in_waiting:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino: {response}")

        # Test LED
        print("\n--- Testing LED ---")
        arduino.write(b"LED\n")
        time.sleep(0.5)
        while arduino.in_waiting:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino: {response}")

    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
    finally:
        if 'arduino' in locals():
            arduino.close()
            print("\nConnection closed")


if __name__ == "__main__":
    test_servos()
