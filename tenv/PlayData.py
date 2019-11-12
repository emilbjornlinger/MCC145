import numpy as np
import pickle
from time import sleep, time

import pyqtgraph as pg
from PyQt5 import QtCore

from acconeer_utils import example_utils
from acconeer_utils.pg_process import PGProcess, PGProccessDiedException

TIME_DELAY = 4

def main():
    loadData()


def loadData():

    # Get data
    file_name = "60si.pkl"
    with open(file_name, "rb") as infile:
        data = pickle.load(infile)
    file_name_meta = "meta60si.pkl"
    with open(file_name_meta, "rb") as infile:
        session_info = pickle.load(infile)

    # Acquire metadata
    range_start = session_info["actual_range_start"]
    range_length = session_info["actual_range_length"]
    sweep_rate = session_info["frequency"]

    pg_updater = PGUpdater(sweep_rate, [range_start, range_start + range_length])
    pg_process = PGProcess(pg_updater)
    pg_process.start()

    processor = PhaseTrackingProcessor(sweep_rate)

    # Set sleep behaviour
    use_sleep = False
    ans = input("Use sleep: [Y/n]")
    if ans == "Y" or ans == "y":
        use_sleep = True

    # Used for sleep time
    per_time = 1 / sweep_rate

    # DEBUG
    counter = 0

    for i in data:
        start_time = time()

        # Process data
        plot_data = processor.process(i)

        counter += 1
        if counter == 997 or counter == 1366:
            input("997 1366 Enter")



        if plot_data is not None:
            pg_process.put_data(plot_data)

        # Handle sleep time
        if use_sleep:
            end_time = time()
            sleep_time = per_time - (end_time - start_time)
            if sleep_time > 0:
                sleep(sleep_time * TIME_DELAY)

class PhaseTrackingProcessor:
    def __init__(self, sweepRate):
        self.f = sweepRate
        self.dt = 1 / self.f

        num_hist_points = int(round(self.f * 3))

        self.lp_vel = 0
        self.last_sweep = None
        self.hist_vel = np.zeros(num_hist_points)
        self.hist_pos = np.zeros(num_hist_points)
        self.sweep_index = 0

    def process(self, sweep):
        n = len(sweep)

        ampl = np.abs(sweep)
        power = ampl*ampl
        if np.sum(power) > 1e-6:
            com = np.sum(np.arange(n)/n * power) / np.sum(power)  # center of mass
        else:
            com = 0

        if self.sweep_index == 0:
            self.lp_ampl = ampl
            self.lp_com = com
            plot_data = None
        else:
            a = self.alpha(0.1, self.dt)
            self.lp_ampl = a*ampl + (1 - a)*self.lp_ampl
            a = self.alpha(0.25, self.dt)
            self.lp_com = a*com + (1-a)*self.lp_com

            com_idx = int(self.lp_com * n)
            delta_angle = np.angle(sweep[com_idx] * np.conj(self.last_sweep[com_idx]))
            vel = self.f * 2.5 * delta_angle / (2*np.pi)

            a = self.alpha(0.1, self.dt)
            self.lp_vel = a*vel + (1 - a)*self.lp_vel

            self.hist_vel = np.roll(self.hist_vel, -1)
            self.hist_vel[-1] = self.lp_vel

            dp = self.lp_vel / self.f
            self.hist_pos = np.roll(self.hist_pos, -1)
            self.hist_pos[-1] = self.hist_pos[-2] + dp

            hist_len = len(self.hist_pos)
            plot_hist_pos = self.hist_pos - self.hist_pos.mean()
            plot_hist_pos_zoom = self.hist_pos[hist_len//2:] - self.hist_pos[hist_len//2:].mean()

            iq_val = np.exp(1j*np.angle(sweep[com_idx])) * self.lp_ampl[com_idx]

            plot_data = {
                "abs": self.lp_ampl,
                "arg": np.angle(sweep),
                "com": self.lp_com,
                "hist_pos": plot_hist_pos,
                "hist_pos_zoom": plot_hist_pos_zoom,
                "iq_val": iq_val,
            }

        self.last_sweep = sweep
        self.sweep_index += 1
        return plot_data

    def alpha(self, tau, dt):
        return 1 - np.exp(-dt/tau)

class PGUpdater:
    def __init__(self, sweepRate, rangeInterval):
        self.sweepRate = sweepRate
        self.interval = rangeInterval

    def setup(self, win):
        win.resize(800, 600)
        win.setWindowTitle("Acconeer phase tracking example")

        self.abs_plot = win.addPlot(row=0, col=0)
        self.abs_plot.showGrid(x=True, y=True)
        self.abs_plot.setLabel("left", "Amplitude")
        self.abs_plot.setLabel("bottom", "Depth (m)")
        self.abs_curve = self.abs_plot.plot(pen=example_utils.pg_pen_cycler(0))
        pen = example_utils.pg_pen_cycler(1)
        pen.setStyle(QtCore.Qt.DashLine)
        self.abs_inf_line = pg.InfiniteLine(pen=pen)
        self.abs_plot.addItem(self.abs_inf_line)

        self.arg_plot = win.addPlot(row=1, col=0)
        self.arg_plot.showGrid(x=True, y=True)
        self.arg_plot.setLabel("bottom", "Depth (m)")
        self.arg_plot.setLabel("left", "Phase")
        self.arg_plot.setYRange(-np.pi, np.pi)
        self.arg_plot.getAxis("left").setTicks(example_utils.pg_phase_ticks)
        self.arg_curve = self.arg_plot.plot(pen=example_utils.pg_pen_cycler(0))
        self.arg_inf_line = pg.InfiniteLine(pen=pen)
        self.arg_plot.addItem(self.arg_inf_line)

        self.iq_plot = win.addPlot(row=1, col=1, title="IQ at line")
        example_utils.pg_setup_polar_plot(self.iq_plot, 0.5)
        self.iq_curve = self.iq_plot.plot(pen=example_utils.pg_pen_cycler())
        self.iq_scatter = pg.ScatterPlotItem(
            brush=pg.mkBrush(example_utils.color_cycler()),
            size=15,
        )
        self.iq_plot.addItem(self.iq_scatter)

        self.hist_plot = win.addPlot(row=0, col=1, colspan=2)
        self.hist_plot.showGrid(x=True, y=True)
        self.hist_plot.setLabel("bottom", "Time (s)")
        self.hist_plot.setLabel("left", "Tracking (mm)")
        self.hist_curve = self.hist_plot.plot(pen=example_utils.pg_pen_cycler())
        self.hist_plot.setYRange(-5, 5)

        self.hist_zoom_plot = win.addPlot(row=1, col=2)
        self.hist_zoom_plot.showGrid(x=True, y=True)
        self.hist_zoom_plot.setLabel("bottom", "Time (s)")
        self.hist_zoom_plot.setLabel("left", "Tracking (mm)")
        self.hist_zoom_curve = self.hist_zoom_plot.plot(pen=example_utils.pg_pen_cycler())
        self.hist_zoom_plot.setYRange(-0.5, 0.5)

        self.smooth_max = example_utils.SmoothMax(self.sweepRate)
        self.first = True

    def update(self, data):
        if self.first:
            self.xs = np.linspace(*self.interval, len(data["abs"]))
            self.ts = np.linspace(-3, 0, len(data["hist_pos"]))
            self.ts_zoom = np.linspace(-1.5, 0, len(data["hist_pos_zoom"]))
            self.first = False

        com_x = (1 - data["com"]) * self.interval[0] + data["com"] * self.interval[1]

        self.abs_curve.setData(self.xs, data["abs"])
        self.abs_plot.setYRange(0, self.smooth_max.update(np.amax(data["abs"])))
        self.abs_inf_line.setValue(com_x)
        self.arg_curve.setData(self.xs, data["arg"])
        self.arg_inf_line.setValue(com_x)
        self.hist_curve.setData(self.ts, data["hist_pos"])
        self.hist_zoom_curve.setData(self.ts_zoom, data["hist_pos_zoom"])
        self.iq_curve.setData([0, np.real(data["iq_val"])], [0, np.imag(data["iq_val"])])
        self.iq_scatter.setData([np.real(data["iq_val"])], [np.imag(data["iq_val"])])

if __name__ == "__main__":
    main()