# Vision-Based Navigation and GPS Emulation

This folder contains a minimal, modular scaffold for a GPS-denied navigation
pipeline using MTF-01 optical flow + ToF data over MAVLink.

## Layout

- `main.py`: entry point (connects, reads, processes, visualizes)
- `config.py`: runtime parameters and defaults
- `io_mavlink.py`: MAVLink connection and message reading helpers
- `processing.py`: velocity and position estimation
- `visualization.py`: live plots and side-panel readout
- `gps_emulation.py`: GPS_INPUT construction and send helpers

## Quick start

```bash
python -m vision_nav.main --port COM11 --baud 115200
```

Adjust port/baud and tuning in `vision_nav/config.py`.

## HTML plotter (Web Serial)

HTML plotter to read your MicoLink optical flow data directly from the serial device
(COM11 @ 115200) using Web Serial, and to show exactly the fields from your
screenshot. The panel has fixed labels for DISTANCE/STRENGTH/PRECISION/STATUS/
FLOW_VEL_X/FLOW_VEL_Y/FLOW_QUAL/FLOW_STA, and the decoder matches
`micolink_reader.py`.

Changes are in `plotter.html`.

How to use it:

```bash
python -m http.server 8000
```

- Open `http://localhost:8000/vision_nav/plotter.html`
- Click "Connect Serial" and select COM11
- Data will plot and update the DATA panel

If you want this to auto-connect specifically to COM11 without selection, we
would need a small local bridge server (Python) instead of the browser's Web
Serial dialog.

## Performance note

Our sensor output is small:

- 100 Hz
- Packet size ~30 bytes
- Data rate ~3 KB/s

Python on Raspberry Pi 4/5 has ample headroom:

- Parse 1 byte ~0.3 us
- Decode packet ~30-50 us
- 100 packets ~5 ms
- CPU load < 1%
