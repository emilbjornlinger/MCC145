# Acconeer Python Exploration Kit

Acconeer Python Exploration Kit is a set of tools and examples for getting started with the Acconeer development kits. By seamlessly feeding radar data to your local machine, it allows you to quickly start exploring the world of Acconeer radar sensor technology. This repository serves as a good starting point both for evaluation purposes and algorithm development in Python.

To run the exploration scripts, you'll need an Acconeer development kit running the streaming or module server supplied with the Acconeer A1 SDK.

This release supports SDK version 1.5.2 or newer.

## Setting up your development kit

For general help on getting started, head over to the [Acconeer website](https://www.acconeer.com/products). There you'll find both a getting started guide and a video showing you how to set up your development kit. There you'll also find the SDK download.

### X111 or X112 (mounted on a Raspberry Pi)

#### Overview

At large, these are the steps you'll need to take:

* Assemble your development kit
* Set up your Raspberry Pi
* Load the Acconeer A1 SDK onto your Raspberry Pi
* Run the streaming server

For a single sensor setup, we recommend plugging the sensor into port 1 for simplicity's sake.

#### Running the streaming server

On your Raspberry Pi, start the streaming server located under `utils` in `AcconeerEvk`:
```
$ cd AcconeerEvk
$ ./utils/acc_streaming_server_rpi_xc112_r2b_xr112_r2b_a111_r2c
```
If you have an X111 kit, the streaming server will instead be named `acc_streaming_server_rpi_xc111_r4a_xr111-3_r1c_a111_r2c`.

Find the IP address of your Raspberry Pi by running `ifconfig` in its terminal.

### XM112

#### Finding the serial port

On Windows, use device manager to find the port which will be listed as `USB Serial Port`. It's most likely `COMx` where `x` is 3 or higher. On Linux, it's likely `/dev/ttyUSBx` where `x` is 0 or higher.

PySerial has a simple tool for listing all ports available:
```
python -m serial.tools.list_ports
```

#### Flashing

For detailed flashing instructions, head over to the [Acconeer website](https://www.acconeer.com/products).

We recommend flashing using BOSSA ([website](http://www.shumatech.com/web/products/bossa), [GitHub](https://github.com/shumatech/BOSSA)). BOSSA 1.9 or newer is supported.

To get into the bootloader:
- Hold down the ERASE button
- Push the NRST button
- Release the NRST button
- Let go of the ERASE button

Now you should be able to flash the module server (`acc_module_server_xm112.bin`). After flashing, press the NRST button to reboot into the flashed software.

If you're on Linux you likely will need to compile BOSSA on your own. In our experience, running Ubuntu 18.04, you will need to install `libreadline-dev` and `libwxgtk3.0-dev` before compiling with `make`. To get everything you need:
```
sudo apt-get install libreadline-dev libwxgtk3.0-dev make build-essential
```

## Setting up your local machine

### Dependencies

Python 3 (developed and tested on 3.6).

Setuptools, NumPy, SciPy, PySerial, matplotlib, PyQtGraph (and PyQt5).

Install all Python dependencies using pip:

```
python -m pip install --user -r requirements.txt
```
_Depending on your environment, you might have to replace `python` with `python3` or `py`._

### Setup

Install the supplied Acconeer utilities module:

```
python setup.py install --user
```

If you're running Linux together with the XM112, you probably need permission to access the serial port. Access is obtained by adding yourself to the dialout group:
```
sudo usermod -a -G dialout your-user-name
```
For the changes to take effect, you will need to log out and in again.

Note: If you have ModemManager installed and running it might try to connect to the XM112, which has proven to cause problems. If you're having issues, try disabling the ModemManager service.

## Running an example on your local machine

Against X111 or X112 (mounted on a Raspberry Pi):
```
python examples/basic.py -s 192.168.1.234
```
Against XM112, autodetecting the serial port:
```
python examples/basic.py -u
```
Against XM112, given a serial port (on Windows):
```
python examples/basic.py -u COM3
```
_Again, depending on your environment, you might have to replace `python` with `python3` or `py`._

Choosing which sensor(s) to be used can be done by adding the argument `--sensor <id 1> [id 2] ...`. The default is the sensor on port 1. This is not applicable for XM112.

Scripts can be terminated by pressing Ctrl-C in the terminal.

## Examples

### Basic

The basic scripts contains a lot of comments guiding you through the steps taken in most example scripts. We recommend taking a look at these scripts before working with the others.

- `basic.py` \
  Basic script for getting data from the radar. **Start here!**
- `basic_continuous.py` \
  Basic script for getting data continuously that serves as the base for most other examples.

### Services

- `power_bin.py` \
  Demonstrates the power bin service.
- `envelope.py` \
  Demonstrates the envelope service.
- `iq.py` \
  Demonstrates the IQ service.

### Detectors

- `distance_peak_fix_threshold.py` \
  Demonstrates the *distance peak* detector.

### Processing

- `breathing.py` \
  An example breathing detection algoritm.
- `phase_tracking.py` \
  Example of tracking relative movements using phase information.
- `presence_detection.py` \
  Shows one way to perform presence detection while ignoring static objects.

### Plotting

- `plot_with_matplotlib.py` \
  Example of how to use matplotlib for plotting IQ data.
- `plot_with_pyqtgraph.py` \
  Example of how to use PyQtGraph for plotting IQ data.