# IC-7300 Web Remote

**Browser-based remote control, live spectrum/waterfall, CW console, and ADIF QSO logging for the Icom IC-7300.**

Developed by **9A3ZI**. The application is a single Python server with an embedded web interface. It communicates with the transceiver through the USB CI-V serial connection and uses WebSockets to update the browser.

> **Status:** Personal/experimental amateur-radio project. Test locally before relying on remote operation. Radio features and available CI-V responses may depend on radio configuration and firmware.

## Features

- Frequency tuning and band selection (including a 4 m shortcut; availability depends on the radio's regional configuration).
- Operating mode and FIL1/FIL2/FIL3 selection.
- Readback and periodic synchronization of radio settings such as TX power, AF/RF gain, squelch, CW speed, preamp, attenuator, NB, NR, ANF, split and tuner.
- Live spectrum/waterfall streamed from IC-7300 CI-V scope data.
- S-meter, TX power and SWR indications.
- Optional browser RX/TX audio through the radio's USB audio device.
- PTT control, CW text transmission, CW stop and a separate CW console with editable macros.
- QSO entry and ADIF logging with callsign, sent/received RST, name and QTH.
- Search, edit and delete QSO records in the web log; backups are created before edits/deletions.
- Log dates shown as **DD/MM/YYYY** in the table, while ADIF stores the standard **YYYYMMDD** value.
- Links between the main control page, CW console and log page.

## Requirements

- Icom IC-7300 and a working USB connection to the computer.
- Windows computer with Python 3.10+ (the project has been used on Windows with Python 3.13).
- The radio's USB serial/CI-V driver and the correct COM port.
- Python packages listed in `requirements.txt`.
- Optional: USB audio input/output devices for browser audio.

Install dependencies:

```powershell
py -m pip install -r requirements.txt
```

If you only need CI-V control and logging, `numpy` and `sounddevice` are optional. The other packages are required.

## Radio configuration

1. Connect the IC-7300 via USB and find its CI-V COM port in Windows Device Manager.
2. Match the radio's CI-V baud rate to the `--baud` option (default **115200**).
3. The default CI-V address is **94h**, configured by `--addr 94`.
4. For CW keying using RTS/DTR, configure **USB Keying (CW)** on the radio to match `--keyline` and verify that **USB SEND** does not inadvertently key the transmitter.
5. For spectrum streaming, the radio must support and provide CI-V scope data. The server requests scope data without intentionally forcing the LCD into the SCOPE screen; actual behavior may vary by radio configuration.

**Important:** Test CW, PTT and USB control with a dummy load or other safe RF setup before using remote transmit functions.

## Run the server

From the project directory:

```powershell
py ICweb.py --port COM3 --baud 115200 --addr 94 --host 127.0.0.1 --web-port 8080 --password "CHANGE_ME"
```

Replace `COM3` with the COM port assigned to your IC-7300. Change the example password.

Open these pages in a browser:

| Page | URL |
| --- | --- |
| Main radio control | `http://localhost:8080/` |
| CW console | `http://localhost:8080/cw` |
| QSO log | `http://localhost:8080/log` |

### Optional audio

List audio devices:

```powershell
py ICweb.py --port COM3 --list-audio
```

Start with selected audio device names or indices:

```powershell
py ICweb.py --port COM3 --password "CHANGE_ME" --audio-in "USB Audio CODEC" --audio-out "USB Audio CODEC"
```

You can adjust received audio gain with `--gain 3.0` (default). Device names must match your Windows audio configuration.

### Other options

```text
--port       Serial port (required), e.g. COM3
--baud       CI-V baud rate; default 115200
--addr       CI-V address in hexadecimal; default 94
--host       Listen address; default 0.0.0.0
--web-port   HTTP(S) port; default 8080
--password   HTTP Basic Authentication password
--audio-in   Optional RX audio device
--audio-out  Optional TX audio device
--gain       Optional RX audio gain; default 3.0
--keyline    RTS or DTR for USB CW keying; default RTS
--list-audio List available audio devices
--cert       TLS certificate file for HTTPS
--key        TLS private-key file for HTTPS
```

## QSO log and macros

The application creates local files beside the Python script:

- `ic7300_log.adi`: QSO records in ADIF format.
- `ic7300_log.adi.bak`: backup produced before log edits/deletions.
- `ic7300_macros.json`: CW macros and saved station-related fields.

These files are ignored by Git in the supplied `.gitignore` to avoid accidentally publishing personal log data or station settings. Back up your ADIF log separately.

The log table shows dates in `DD/MM/YYYY` format. When editing a QSO, enter the date in `YYYYMMDD` format as indicated by the editor.

## Security and remote access

**Do not expose the default server directly to the public internet.** This application can change radio settings and trigger transmission.

- The default listen address is `0.0.0.0`, which can expose the server to other devices on your network. Use `--host 127.0.0.1` for local-only access.
- Set a strong `--password`. The built-in authentication uses HTTP Basic Authentication; without HTTPS, credentials are not encrypted in transit.
- For remote use, prefer a properly configured private VPN and/or HTTPS with suitable access controls. Do not forward port 8080 from your router directly to the internet.
- Never commit private keys, credentials, or real QSO log data.
- Respect applicable amateur-radio regulations and station-control requirements.

## Project structure

```text
IC7300-Web-Remote/
├── ICweb.py          # Python CI-V / HTTP / WebSocket server and embedded web UI
├── README.md         # Project documentation
├── requirements.txt  # Python dependencies
└── .gitignore        # Local/private files excluded from Git
```

## Troubleshooting

- **No radio connection:** Verify COM port, baud rate, USB driver and CI-V address; close other software holding the serial port.
- **Web UI connects but values do not update:** Verify the radio is responding to CI-V read requests.
- **No waterfall:** Check CI-V scope output and radio configuration. The application avoids forcing the LCD to switch away from its current display.
- **No browser audio:** Install optional audio packages, select the correct USB audio devices and verify Windows sound permissions.
- **Windows `WinError 10054`:** An isolated TCP/WebSocket reset during page refresh or disconnect is usually transient. Investigate if repeated or accompanied by loss of control.

## Contributing

Issues, reproducible bug reports and pull requests are welcome. Please include the Python version, operating system, CI-V configuration and steps to reproduce. Remove passwords, callsigns from private logs and other sensitive data from diagnostics.

## License

**No license has been selected yet.** Add a `LICENSE` file before inviting reuse or redistribution under an open-source license (for example, MIT if that is your choice). Until then, the repository does not grant general reuse rights merely by being public.
