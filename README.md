# Galaxy Buds4 Pro Head Tracking for Opentrack

Reads orientation data (360 Audio / Game Rotation Vector) directly from Samsung Galaxy Buds (tested on **Galaxy Buds4 Pro**, compatible with Buds2 / Buds3 / Pro models) over Bluetooth RFCOMM and streams 3-DOF head tracking via UDP to **Opentrack** on Linux and Windows.

Designed for flight simulators like **Microsoft Flight Simulator 2024** (including Linux/Steam Proton + Wayland setups) where mouse-emulation head trackers break cockpit clicking.

## What's New in `buds4-headtracking-v2.py`

* **Strict CRC-16 Validation:** Drops corrupted Bluetooth frames instead of forwarding garbage angles (eliminates sudden 180° cockpit snaps).
* **Orientation Spike Filter (`--max-jump`):** Rejects single-packet IMU glitches while recovering automatically if the sensor resets its reference.
* **3-DOF Roll Support (`--roll`):** Computes true head roll without pitch crosstalk, scalable for subtle cockpit leaning.
* **Optional One Euro Smoothing (`--smooth`):** Low-latency adaptive smoothing filter for steady instrument reading.
* **Auto-Reconnect & Sensor Recovery:** Automatically reconnects on Bluetooth drops and re-attaches if the earbud IMU reports `DATA_STUCK`.
* **Wear Detection Pause:** Freezes tracking automatically when earbuds are removed (`DATA_WEAR_OFF`) and recenters when put back in (`DATA_WEAR_ON`).
* **Wayland-Friendly Global Hotkeys:** Supports `SIGUSR1` (recenter) and `SIGUSR2` (pause/resume) so you can bind them to a Stream Deck or desktop shortcuts without losing game focus.

## Requirements

* Python 3.8+ (uses standard library only, no `pip` dependencies required)
* [Opentrack](https://github.com/opentrack/opentrack) (Linux Flatpak: `flathub org.opentrack.OpenTrack`)
* Earbuds paired and connected to your PC via Classic Bluetooth

## Quick Start

1. **Find your earbuds' Bluetooth MAC address (Linux):**
~~~bash
bluetoothctl devices
~~~

2. **Run the tracker:**
~~~bash
python3 buds4-headtracking-v2.py AA:BB:CC:DD:EE:FF
~~~
   With subtle roll enabled (recommended for flight sims):
~~~bash
python3 buds4-headtracking-v2.py AA:BB:CC:DD:EE:FF --roll 0.4
~~~

3. **Calibrate the pitch axis (first-time setup):**
   Look straight at the center of your monitor, press `u`, and lift your head clearly upward within 3 seconds. The measured axis is saved to `axes.json` so looking up at overhead panels stays level regardless of how the earbuds sit in your ears.

## CLI Options

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--roll N` | `0.0` | Roll scale (`0.0` = off, `0.4` = subtle, `1.0` = 1:1) |
| `--smooth` | `off` | Enable One Euro adaptive smoothing (optional alongside Opentrack Accela) |
| `--max-jump N` | `45.0` | Reject single-packet orientation jumps above `N` degrees (`0` = off) |
| `--yaw-only` | `off` | Send yaw only (pitch and roll forced to `0`) |
| `--channel N` | auto | Manually specify RFCOMM channel if SDP auto-discovery fails |
| `--no-reconnect` | `off` | Exit instead of waiting to reconnect on Bluetooth drop |
| `--quiet` | `off` | Disable live terminal status line |
| `--trace` | `off` | Print raw sent/received hex frames |
| `--seconds N` | `0` | Exit after `N` seconds (`0` = run indefinitely) |

## Controls

**In terminal:**
* `r` — Recenter (look straight ahead)
* `p` — Pause / resume tracking (freezes camera for clicking cockpit instruments)
* `u` — Calibrate pitch/up axis (3-second window)
* `Ctrl+C` — Detach sensor cleanly and quit

**Global triggers (works in-game on Wayland / X11):**
Bind these to a Stream Deck, macro pad, or GNOME custom shortcuts:
~~~bash
# Recenter
pkill -USR1 -f buds4-headtracking-v2

# Pause / Resume
pkill -USR2 -f buds4-headtracking-v2
~~~
*(On Windows, `Ctrl+Alt+R` recenters globally.)*

## Opentrack Configuration (MSFS 2024 on Linux / Proton)

1. **Input:** Select `UDP over network` -> open settings (hammer icon) -> set port to `4242`.
2. **Filter:** `Accela` (default works well; only enable `--smooth` in the script if you still notice micro-jitter).
3. **Output:** Select `Wine` -> open settings -> point to your MSFS 2024 Proton prefix (e.g. `~/.steam/steam/steamapps/compatdata/2537590/pfx`).
4. Click **Start** in Opentrack and launch the sim.

## Credits

* Samsung 360 Audio RFCOMM protocol reverse-engineered by [GalaxyBudsClient](https://github.com/timschneeb/GalaxyBudsClient) (`timschneeb`).
* Forked and expanded for Galaxy Buds4 Pro and flight simulation use.
