import asyncio
import math
import time
import sys
import threading
import serial

from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pyrplidar import PyRPlidar


# ============================================================
# CONFIGURATION
# ============================================================

UART_PORT = "/dev/serial0"
BAUD_RATE = 115200

DRIVE_SPEED = 3
ROTATE_SPEED = 2

ALIGNMENT_SCANS = 2
LIDAR_TIMEOUT = 1.0

W, H = 800, 800
ZOOM = 1
STEP = 45

# ============================================================
# SHARED STATE
# ============================================================

latestScanData = []
bestDirection = None

scanGeneration = 0
lastScanTime = 0

robotState = "stopped"

stateLock = threading.Lock()
lidarStopEvent = threading.Event()

lidarDevice = None
lidarLock = threading.Lock()


# ============================================================
# UART COMMUNICATION
# ============================================================

def connectPico():
    try:
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=BAUD_RATE,
            timeout=0
        )

        time.sleep(0.2)

        print("Connected to Pico.")
        return ser

    except serial.SerialException as e:
        print(f"Failed to connect to Pico: {e}")
        return None


def encodeCommand(speed=0, rotate=False, stop=False):

    command = 0b101

    command |= (speed & 0b111) << 5

    if rotate:
        command |= 1 << 4

    if stop:
        command |= 1 << 3

    return command


def sendCommand(ser, speed=0, rotate=False, stop=False):

    if ser is None or not ser.is_open:
        return

    command = encodeCommand(speed, rotate, stop)

    ser.write(bytes([command]))
    ser.flush()

    print(f"UART: {command:08b}")


def stopRobot(ser):
    sendCommand(ser, stop=True)


def rotateRobot(ser):
    sendCommand(ser, speed=ROTATE_SPEED, rotate=True)


def moveForward(ser):
    sendCommand(ser, speed=DRIVE_SPEED)


# ============================================================
# LIDAR COORDINATE CONVERSION
# ============================================================

def toScreenUnits(x, L=W, maxDist=4000):
    return int((x / maxDist) * (L // 2) * ZOOM)


def polarToCartesian(r, t):

    x = r * math.cos(math.radians(t))
    y = r * math.sin(math.radians(t))

    return Point(x, y)


def cwPolarToCartesian(r, t):
    return polarToCartesian(r, -t)


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def toScreen(self):
        return W // 2 + self.x, H // 2 - self.y


# ============================================================
# LIDAR WORKER (Updated with Forward Bias & Hysteresis)
# ============================================================

def lidarStreamWorker():

    global latestScanData, bestDirection
    global scanGeneration, lastScanTime
    global lidarDevice

    lidar = PyRPlidar()

    try:

        lidar.connect(
            port="/dev/ttyUSB0",
            baudrate=460800,
            timeout=3
        )

        with lidarLock:
            lidarDevice = lidar

        lidar.set_motor_pwm(500)
        time.sleep(2)

        if lidarStopEvent.is_set():
            return

        scanGenerator = lidar.force_scan()

        pointsToLoad = []

        polarHistogram = {
            i: (0, 0) for i in range(8)
        }

        prevAngle = None

        print("LiDAR connected. Beginning data capture...")

        for scan in scanGenerator():

            if lidarStopEvent.is_set():
                break

            # Collect visualization points

            screenR = toScreenUnits(scan.distance)

            coords = cwPolarToCartesian(
                screenR,
                scan.angle
            )

            pointsToLoad.append(coords.toScreen())

            # Convert clockwise LiDAR angle to CCW angle

            ccwAngle = (-scan.angle) % 360

            binIndex = int(
                ((ccwAngle + STEP / 2) % 360) // STEP
            ) % 8

            total_distance, count = polarHistogram[binIndex]

            polarHistogram[binIndex] = (
                total_distance + scan.distance,
                count + 1
            )

            # Detect completion of a full revolution

            if prevAngle is not None and scan.angle < prevAngle:

                totalPoints = sum(
                    polarHistogram[i][1]
                    for i in range(8)
                )

                if totalPoints > 0:
                    # Calculate averages for all bins
                    binAverages = {}
                    for i in range(8):
                        total, count = polarHistogram[i]
                        binAverages[i] = (total / count) if count > 0 else 0

                    # FRONT BIAS & HYSTERESIS:
                    # Get current front distance (Bin 0)
                    frontAvg = binAverages.get(0, 0)
                    
                    # If the front is safe (> 1000mm), strongly favor staying forward
                    # unless another direction is vastly better (+300mm buffer).
                    SAFE_FRONT_THRESHOLD = 1000 
                    HYSTERESIS_BUFFER = 300

                    with stateLock:
                        current_best = bestDirection

                    current_best_bin = (current_best // STEP) if current_best is not None else 0

                    if frontAvg >= SAFE_FRONT_THRESHOLD:
                        currBestDirection = 0
                    else:
                        bestAvg = -1
                        currBestDirection = 0
                        for i in range(8):
                            currAvg = binAverages[i]
                            # Apply hysteresis bonus to the currently chosen direction to prevent flapping
                            if i == current_best_bin:
                                currAvg += HYSTERESIS_BUFFER

                            if currAvg > bestAvg:
                                bestAvg = currAvg
                                currBestDirection = i

                    newBestDirection = currBestDirection * STEP

                else:
                    newBestDirection = 0

                # Publish the completed scan atomically

                with stateLock:

                    bestDirection = newBestDirection

                    latestScanData = list(pointsToLoad)

                    scanGeneration += 1
                    lastScanTime = time.monotonic()

                # Reset for next revolution

                pointsToLoad.clear()

                for i in range(8):
                    polarHistogram[i] = (0, 0)

            prevAngle = scan.angle

    except Exception as e:

        if not lidarStopEvent.is_set():
            print(
                f"\nCRITICAL LIDAR ERROR: {e}",
                file=sys.stderr
            )

    finally:

        print("Cleaning up LiDAR...")

        try:
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()

        except Exception:
            pass

        with lidarLock:
            lidarDevice = None

        print("LiDAR disconnected.")


# ============================================================
# AUTONOMOUS NAVIGATION (Updated with Stability Buffer)
# ============================================================

async def navigationLoop(ser):

    global robotState

    alignedScans = 0
    misalignedScans = 0

    lastGeneration = -1

    print("Autonomous navigation started.")

    while True:

        # Obtain a consistent LiDAR snapshot

        with stateLock:

            direction = bestDirection
            generation = scanGeneration
            scanTime = lastScanTime

        # Safety: stop if LiDAR data becomes stale

        if (
            scanTime == 0 or
            time.monotonic() - scanTime > LIDAR_TIMEOUT
        ):

            if robotState != "stopped":

                stopRobot(ser)
                robotState = "stopped"

                print("LiDAR timeout. Robot stopped.")

            alignedScans = 0
            misalignedScans = 0

            await asyncio.sleep(0.05)
            continue

        # Only count new completed scans

        if generation != lastGeneration:

            lastGeneration = generation

            if direction == 0:

                alignedScans += 1
                misalignedScans = 0

            else:

                misalignedScans += 1
                alignedScans = 0

        # ----------------------------------------------------
        # STABLE STATE MACHINE
        # ----------------------------------------------------

        STABILITY_BUFFER = max(ALIGNMENT_SCANS, 3)

        # Currently driving, but direction is no longer aligned

        if (
            robotState == "driving" and
            misalignedScans >= STABILITY_BUFFER
        ):

            stopRobot(ser)

            robotState = "stopped"

            print(
                f"Direction changed to {direction} degrees. "
                "Stopping to realign."
            )

        # Currently rotating, and direction is now aligned

        elif (
            robotState == "rotating" and
            alignedScans >= ALIGNMENT_SCANS
        ):

            stopRobot(ser)

            robotState = "stopped"

            print("Aligned with forward direction.")

        # Currently stopped: decide what to do next

        if robotState == "stopped":

            if alignedScans >= ALIGNMENT_SCANS:

                moveForward(ser)

                robotState = "driving"

                print("DRIVING FORWARD.")

            elif misalignedScans >= STABILITY_BUFFER:

                rotateRobot(ser)

                robotState = "rotating"

                print(
                    f"ROTATING. Best direction: "
                    f"{direction} degrees."
                )

        await asyncio.sleep(0.05)


# ============================================================
# FASTAPI LIFESPAN
# ============================================================

@asynccontextmanager
async def lifespan(app: FastAPI):

    global robotState

    print("Starting autonomous robot system...")

    lidarStopEvent.clear()

    pico = connectPico()

    # Start LiDAR worker

    lidarTask = asyncio.create_task(
        asyncio.to_thread(lidarStreamWorker)
    )

    navigationTask = None

    if pico is not None:

        stopRobot(pico)

        robotState = "stopped"

        navigationTask = asyncio.create_task(
            navigationLoop(pico)
        )

    try:
        yield

    finally:

        print("Shutting down robot system...")

        # Stop robot first

        if pico is not None:

            stopRobot(pico)
            robotState = "stopped"

        # Stop navigation coroutine

        if navigationTask is not None:

            navigationTask.cancel()

            try:
                await navigationTask

            except asyncio.CancelledError:
                pass

        # Request LiDAR shutdown

        lidarStopEvent.set()

        # Attempt to interrupt the LiDAR worker

        with lidarLock:
            device = lidarDevice

        if device is not None:

            try:
                device.stop()
                device.set_motor_pwm(0)

            except Exception:
                pass

        # Allow worker time to clean up

        try:
            await asyncio.wait_for(
                asyncio.shield(lidarTask),
                timeout=3.0
            )

        except asyncio.TimeoutError:
            print("LiDAR worker did not exit within 3 seconds.")

        except Exception as e:
            print(f"LiDAR shutdown error: {e}")

        # Close UART

        if pico is not None:

            stopRobot(pico)
            pico.close()

        print("Shutdown complete.")


app = FastAPI(lifespan=lifespan)


# ============================================================
# WEBPAGE
# ============================================================

html = """<!DOCTYPE html>
<html>

<head>

    <title>LiDAR Autonomous Robot</title>

    <style>

        body {
            background: #111;
            color: white;
            text-align: center;
            font-family: sans-serif;
        }

        canvas {
            background: #000;
            border: 2px solid #333;
            margin-top: 10px;
        }

        #status {
            margin-top: 10px;
            font-size: 18px;
        }

    </style>

</head>

<body>

    <h1>LiDAR Autonomous Robot</h1>

    <div id="status">Connecting...</div>

    <canvas
        id="radarCanvas"
        width="800"
        height="800">
    </canvas>

    <script>

        const canvas = document.getElementById("radarCanvas");
        const ctx = canvas.getContext("2d");

        const cx = canvas.width / 2;
        const cy = canvas.height / 2;

        const status = document.getElementById("status");

        const ws = new WebSocket(
            `ws://${window.location.host}/ws`
        );

        ws.onopen = () => {
            status.innerText = "Connected";
        };

        ws.onclose = () => {
            status.innerText = "Disconnected";
        };

        ws.onmessage = (event) => {

            const data = JSON.parse(event.data);

            drawRadar(
                data.points,
                data.best_angle,
                data.robot_state
            );

        };

        function drawRadar(points, bestAngle, robotState) {

            ctx.clearRect(
                0, 0,
                canvas.width,
                canvas.height
            );

            // Grid rings

            ctx.strokeStyle = "#222";
            ctx.lineWidth = 1;

            for (let r = 50; r < cx; r += 50) {

                ctx.beginPath();
                ctx.arc(cx, cy, r, 0, 2 * Math.PI);
                ctx.stroke();

            }

            // Forward reference line (horizontal right)

            ctx.strokeStyle = "#00aa00";
            ctx.lineWidth = 1;

            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + 350, cy);
            ctx.stroke();

            // Robot center

            ctx.fillStyle = "#00ff00";

            ctx.beginPath();
            ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
            ctx.fill();

            // LiDAR points

            ctx.fillStyle = "#ff0000";

            points.forEach(p => {

                ctx.beginPath();

                ctx.arc(
                    p.x,
                    p.y,
                    1.5,
                    0,
                    2 * Math.PI
                );

                ctx.fill();

            });

            // Best direction line

            if (bestAngle !== null) {

                const rad = bestAngle * Math.PI / 180;

                const targetX = cx + 300 * Math.cos(rad);
                const targetY = cy - 300 * Math.sin(rad);

                ctx.strokeStyle = "#0000ff";
                ctx.lineWidth = 3;

                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(targetX, targetY);
                ctx.stroke();

            }

            // Display navigation state

            status.innerText =
                "Robot state: " + robotState +
                " | Best direction: " +
                (bestAngle === null ? "Waiting" : bestAngle + "°");

        }

    </script>

</body>

</html>
"""


# ============================================================
# HTTP ENDPOINT
# ============================================================

@app.get("/")
async def get():
    return HTMLResponse(html)


# ============================================================
# WEBSOCKET ENDPOINT
# ============================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):

    await websocket.accept()

    try:

        while True:

            with stateLock:

                points = list(latestScanData)
                direction = bestDirection

            payload = {

                "points": [
                    {"x": p[0], "y": p[1]}
                    for p in points
                ],

                "best_angle": direction,
                "robot_state": robotState

            }

            await websocket.send_json(payload)

            await asyncio.sleep(0.05)

    except WebSocketDisconnect:

        print("WebSocket disconnected")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    import uvicorn

    uvicorn.run(
        "server:app",
        host="0.0.0.0",
        port=8000,
        reload=False
    )
