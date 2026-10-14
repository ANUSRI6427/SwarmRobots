
import math
import os
import re
import shutil
import subprocess
import threading
import time

# ---------- MineSwarm topics ----------
SENSOR_TOPIC = "/robot1/ultrasonic/scan"
CMD_TOPIC = "/model/robot1/cmd_vel"

# ---------- Obstacle avoidance settings ----------
FORWARD_SPEED = 0.25
TURN_SPEED = 1.10
STOP_DISTANCE = 1.30       # Start turning within 1.30 metres
CLEAR_DISTANCE = 1.45      # Resume only when the path is clear
TURN_DURATION = 0.8
SENSOR_TIMEOUT = 2.0
LOOP_HZ = 5
MAX_RANGE = 4.0

latest_range = MAX_RANGE
last_sensor_time = 0.0
lock = threading.Lock()
running = True
turn_direction = 1.0

# Find Gazebo's Windows command.
GZ = shutil.which("gz.bat") or shutil.which("gz") or "gz"


def make_command(args):
    """Prepare a Windows-compatible Gazebo command."""
    return subprocess.list2cmdline([GZ] + args)


def publish_velocity(linear, angular):
    """Send velocity commands to the robot."""
    message = (
        f"linear: {{x: {linear:.3f}}}, "
        f"angular: {{z: {angular:.3f}}}"
    )

    command = make_command([
        "topic", "-t", CMD_TOPIC,
        "-m", "gz.msgs.Twist", "-p", message
    ])

    try:
        result = subprocess.run(
            command,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2,
            check=False
        )
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def sensor_reader():
    """Read scalar range measurements published by Gazebo."""
    global latest_range, last_sensor_time

    command = make_command(["topic", "-e", "-t", SENSOR_TOPIC])

    while running:
        process = None

        try:
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            print(f"\nListening to {SENSOR_TOPIC}")

            for line in process.stdout:
                if not running:
                    break

                # Your Gazebo sensor publishes:
                # ranges: 1.1748173236846924
                match = re.search(
                    r"\branges\s*:\s*"
                    r"([-+]?(?:\d+\.?\d*|\.\d+)"
                    r"(?:[eE][-+]?\d+)?)",
                    line
                )

                if not match:
                    continue

                try:
                    distance = float(match.group(1))
                except ValueError:
                    continue

                if math.isfinite(distance) and 0.03 <= distance <= MAX_RANGE:
                    with lock:
                        latest_range = distance
                        last_sensor_time = time.monotonic()

        except Exception as exc:
            print(f"\nSensor connection error: {exc}")

        finally:
            if process is not None and process.poll() is None:
                process.terminate()

        if running:
            time.sleep(1)


def main():
    global running, turn_direction

    print("\nMineSwarm obstacle avoidance started")
    print("Press Ctrl+C to stop.")
    print(f"Stop distance: {STOP_DISTANCE:.2f} m")
    print(f"Clear distance: {CLEAR_DISTANCE:.2f} m")

    threading.Thread(target=sensor_reader, daemon=True).start()

    mode = "forward"
    turn_until = 0.0

    try:
        while True:
            now = time.monotonic()

            with lock:
                distance = latest_range
                last_update = last_sensor_time

            if last_update == 0.0 or now - last_update > SENSOR_TIMEOUT:
                # Never move if the sensor is disconnected or stale.
                linear, angular = 0.0, 0.0
                status = "STOP: sensor unavailable"
                mode = "waiting"

            elif mode == "turn":
                if now < turn_until:
                    linear, angular = 0.0, TURN_SPEED * turn_direction
                    status = "Avoiding obstacle"

                elif distance < CLEAR_DISTANCE:
                    # The path is still blocked. Continue turning
                    # in the same direction and check the sensor again.
                    turn_until = now + TURN_DURATION
                    linear, angular = 0.0, TURN_SPEED * turn_direction
                    status = "Path blocked - keep turning"

                else:
                    mode = "forward"
                    linear, angular = FORWARD_SPEED, 0.0
                    status = "Path clear - moving forward"

            elif distance <= STOP_DISTANCE:
                mode = "turn"
                turn_until = now + TURN_DURATION
                linear, angular = 0.0, TURN_SPEED * turn_direction
                status = "OBSTACLE DETECTED"

            else:
                linear, angular = FORWARD_SPEED, 0.0
                status = "Moving forward"

            sent = publish_velocity(linear, angular)

            print(
                f"\r{status:29s} | "
                f"Range: {distance:.2f} m | "
                f"v={linear:.2f} | w={angular:.2f} | "
                f"cmd={'OK' if sent else 'FAILED'}     ",
                end="",
                flush=True
            )

            time.sleep(1.0 / LOOP_HZ)

    except KeyboardInterrupt:
        print("\nStopping MineSwarm...")

    finally:
        running = False
        publish_velocity(0.0, 0.0)


if __name__ == "__main__":
    main()
