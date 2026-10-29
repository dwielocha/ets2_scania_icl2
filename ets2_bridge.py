#!/usr/bin/env python3
"""
ETS2 -> Scania ICL bridge.

Reads the ets2-telemetry-server JSON and sends one text line per update to the
Arduino Mega:
    d<speed km/h x10>,<rpm>,<fuel % x10>,<flags>,<odometer km x10>,<water C>,<oil C>\n

flags: 1 left indicator, 2 right indicator, 4 high beam, 8 low beam,
       16 parking brake, 32 parking lights (hazards = left + right)

Usage:
    pip install pyserial
    python ets2_bridge.py COM5            # real run, indicators follow the lever, Arduino flashes them
    python ets2_bridge.py COM5 --lamp     # indicators follow the game's lamp field (if your server has it)
    python ets2_bridge.py --dry           # only print values, no serial (checks the JSON)
    python ets2_bridge.py COM5 --test     # fake data, no game needed (sweeps speed/rpm/fuel, cycles lights)
"""
import json
import sys
import time
import urllib.request

URL = "http://localhost:25555/api/ets2/telemetry"
BAUD = 115200
RATE_HZ = 20

# last good values, sent while the game is not connected
last = {"odo": 0, "water": 85, "oil": 85}


def fetch():
    with urllib.request.urlopen(URL, timeout=0.5) as r:
        return json.load(r)


def blinker(truck, side, use_lever):
    lever = truck.get(f"blinker{side}On", False)
    lamp = truck.get(f"blinker{side}Active", lever)
    return bool(lever if use_lever else lamp)


def convert(d, use_lever):
    """Return (speed_x10, rpm, fuel_x10, flags, odo_x10, water, oil)."""
    truck = d["truck"]
    if not d["game"]["connected"]:
        return 0, 0, 0, 0, last["odo"], last["water"], last["oil"]

    speed = abs(float(truck["speed"]))           # km/h in the Funbit server
    rpm = float(truck["engineRpm"])
    cap = float(truck["fuelCapacity"]) or 1.0
    fuel_pct = 100.0 * float(truck["fuel"]) / cap

    flags = 0
    flags |= 1 if blinker(truck, "Left", use_lever) else 0
    flags |= 2 if blinker(truck, "Right", use_lever) else 0
    flags |= 4 if truck.get("lightsBeamHighOn") else 0
    flags |= 8 if truck.get("lightsBeamLowOn") else 0
    flags |= 16 if truck.get("parkBrakeOn") else 0
    flags |= 32 if truck.get("lightsParkingOn") else 0

    last["odo"] = int(round(float(truck.get("odometer", 0)) * 10))
    last["water"] = int(round(float(truck.get("waterTemperature", last["water"]))))
    last["oil"] = int(round(float(truck.get("oilTemperature", last["oil"]))))

    return (
        max(0, min(2500, int(round(speed * 10)))),
        max(0, min(8000, int(round(rpm)))),
        max(0, min(1000, int(round(fuel_pct * 10)))),
        flags,
        last["odo"],
        last["water"],
        last["oil"],
    )


def test_values(t):
    """Fake data for testing the chain without the game."""
    ph = (t % 30) / 30.0
    tri = 1 - abs(2 * ph - 1)                    # 0 -> 1 -> 0 over 30 s
    speed = tri * 100
    rpm = 600 + tri * 1400
    fuel = 100 - (t % 60) / 60 * 100
    seq = [0, 1, 2, 3, 4, 8, 16, 32]             # none, L, R, hazards, high, low, park brake, park lights
    flags = seq[int(t // 3) % len(seq)]
    return int(speed * 10), int(rpm), int(fuel * 10), flags, 1234567, 85, 90


def main():
    dry = "--dry" in sys.argv
    test = "--test" in sys.argv
    use_lever = "--lamp" not in sys.argv     # lever state by default, the Arduino does the flashing
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    ser = None
    if not dry:
        import serial
        port = args[0] if args else "COM5"
        ser = serial.Serial(port, BAUD, timeout=0)
        # The Mega resets when the port opens.
        time.sleep(4)

    period = 1.0 / RATE_HZ
    start = time.time()
    prev_flags = [None]
    while True:
        t0 = time.time()
        try:
            if test:
                data = {"truck": {}}
                s, r, f, fl, odo, w, o = test_values(time.time() - start)
                if fl != prev_flags[0]:
                    prev_flags[0] = fl
                    print(f"test: flags {fl:06b}")
            else:
                data = fetch()
                s, r, f, fl, odo, w, o = convert(data, use_lever)
            if dry:
                t = data["truck"]
                print(
                    f"speed {s/10:5.1f}  rpm {r:5d}  fuel {f/10:5.1f}%  flags {fl:06b}  "
                    f"odo {odo/10:9.1f}  water {w}  oil {o}  | "
                    f"blinkerL on/active: {t.get('blinkerLeftOn')}/{t.get('blinkerLeftActive')}  "
                    f"R: {t.get('blinkerRightOn')}/{t.get('blinkerRightActive')}"
                )
            else:
                ser.write(f"d{s},{r},{f},{fl},{odo},{w},{o}\n".encode("ascii"))
        except Exception as e:                    # server not running, game closed, ...
            print("telemetry error:", e)
            time.sleep(1)

        if ser is not None:
            ser.read(4096)                        # drain Arduino output
        time.sleep(max(0, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
