import numpy as np
import pickle
from time import sleep, time
import math

from acconeer_utils.clients import UARTClient, SPIClient, SocketClient
from acconeer_utils.clients import configs
from acconeer_utils import example_utils
from acconeer_utils.pg_process import PGProcess, PGProccessDiedException

DIST_THRESHOLD = 0.05
AMPL_THRESHOLD = 0.01
DEBUG = False
DEBUG_PRINT = False


def main():
    # Set sleep behaviour
    use_sleep = False
    ans = input("Use sleep: [Y/n]")
    if ans == "Y" or ans == "y":
        use_sleep = True

    load_data(use_sleep, DEBUG, DEBUG_PRINT, AMPL_THRESHOLD, DIST_THRESHOLD, "30mi.pkl", "meta30mi.pkl")
    #sensor_read()


def sensor_read():
    args = example_utils.ExampleArgumentParser(num_sens=1).parse_args()
    example_utils.config_logging(args)

    if args.socket_addr:
        client = SocketClient(args.socket_addr)
    elif args.spi:
        client = SPIClient()
    else:
        port = args.serial_port or example_utils.autodetect_serial_port()
        client = UARTClient(port)

    sensor_config = get_sensor_config()
    sensor_config.sensor = args.sensors

    # Extract metadata
    session_info = client.setup_session(sensor_config)
    range_start = session_info["actual_range_start"]
    range_length = session_info["actual_range_length"]
    #sweep_rate = session_info["frequency"]
    data_length = session_info["data_length"]

    client.start_streaming()

    interrupt_handler = example_utils.ExampleInterruptHandler()
    print("Press Ctrl-C to end session")

    # Instantiate customProcess
    custom_processor = customProcess(range_length, range_start, AMPL_THRESHOLD, DIST_THRESHOLD, data_length, -1)

    processor = PhaseTrackingProcessor(sensor_config.sweep_rate)
    while not interrupt_handler.got_signal:
        info, sweep = client.get_next()
        plot_data = processor.process(sweep)

        if plot_data is not None:
            try:
                #print(np.amax(plot_data["abs"]))
                if custom_processor.process(plot_data):
                    person_counter = custom_processor.person_counter
                    if person_counter == 1:
                        print("1 person in the room")
                        #input("Enter")
                    else:
                        print(person_counter, " persons in the room")
                        #input("Enter")

            except PGProccessDiedException:
                break

    print("Disconnecting...")
    client.disconnect()


def get_sensor_config():
    config = configs.IQServiceConfig()
    config.range_interval = [0.3, 0.8]
    config.sweep_rate = 70
    config.gain = 0.7
    return config


def load_data(use_sleep, debug, debug_print, ampl_threshold, dist_threshold, filename = "", metafilename = ""):
    # get data
    file_name = filename
    with open(file_name, "rb") as infile:
        data = pickle.load(infile)
    file_name_meta = metafilename
    with open(file_name_meta, "rb") as infile:
        session_info = pickle.load(infile)

    # Extract metadata
    range_start = session_info["actual_range_start"]
    range_length = session_info["actual_range_length"]
    sweep_rate = session_info["frequency"]
    data_length = session_info["data_length"]

    processor = PhaseTrackingProcessor(sweep_rate)


    per_time = 1 / sweep_rate

    # Instantiate customProcess
    custom_processor = customProcess(range_length, range_start, ampl_threshold, dist_threshold, data_length, -1)

    counter = 0
    return_counter = 0

    for i in data:
        start_time = time()
        plot_data = processor.process(i)
        counter += 1

        if plot_data is not None:
            if debug_print:
                abs = plot_data["abs"]
                maxVal = np.amax(abs)
                print("MaxValue:", maxVal)
            else:

                #DEBUG

                if debug:
                    returnVal = custom_processor.process(plot_data)
                    if returnVal != None:
                        print("Iteration: ", counter)
                        input("Enter")
                    if returnVal:
                        person_counter = custom_processor.person_counter
                        if person_counter == 1:
                            print("1 person in the room")
                        else:
                            print(person_counter, " persons in the room")
                else:
                    if custom_processor.process(plot_data):
                        person_counter = custom_processor.person_counter
                        return_counter = person_counter
                        if person_counter == 1:
                            print("1 person in the room")
                        else:
                            print(person_counter, " persons in the room")

        # Handle sleep time
        if use_sleep:
            end_time = time()
            sleep_time = per_time - (end_time - start_time)
            if sleep_time > 0:
                sleep(sleep_time)

    print("Counter finish:", counter)
    return return_counter

class PhaseTrackingProcessor:
    def __init__(self, sweep_rate):
        self.f = sweep_rate
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
        power = ampl * ampl
        if np.sum(power) > 1e-6:
            com = np.sum(np.arange(n) / n * power) / np.sum(power)  # center of mass
        else:
            com = 0

        if self.sweep_index == 0:
            self.lp_ampl = ampl
            self.lp_com = com
            plot_data = None
        else:
            a = self.alpha(0.1, self.dt)
            self.lp_ampl = a * ampl + (1 - a) * self.lp_ampl
            a = self.alpha(0.25, self.dt)
            self.lp_com = a * com + (1 - a) * self.lp_com

            com_idx = int(self.lp_com * n)
            delta_angle = np.angle(sweep[com_idx] * np.conj(self.last_sweep[com_idx]))
            vel = self.f * 2.5 * delta_angle / (2 * np.pi)

            a = self.alpha(0.1, self.dt)
            self.lp_vel = a * vel + (1 - a) * self.lp_vel

            self.hist_vel = np.roll(self.hist_vel, -1)
            self.hist_vel[-1] = self.lp_vel

            dp = self.lp_vel / self.f
            self.hist_pos = np.roll(self.hist_pos, -1)
            self.hist_pos[-1] = self.hist_pos[-2] + dp

            hist_len = len(self.hist_pos)
            plot_hist_pos = self.hist_pos - self.hist_pos.mean()
            plot_hist_pos_zoom = self.hist_pos[hist_len // 2:] - self.hist_pos[hist_len // 2:].mean()

            iq_val = np.exp(1j * np.angle(sweep[com_idx])) * self.lp_ampl[com_idx]

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
        return 1 - np.exp(-dt / tau)


class customProcess:
    def __init__(self, range_interval, range_start, ampl_threshold, distance_threshold, data_length, in_value, person_counter=0):
        self.presence_array = []
        self.range_interval = range_interval
        self.range_start = range_start
        self.ampl_threshold = ampl_threshold
        self.distance_threshold = distance_threshold
        self.data_length = data_length
        self.in_value = in_value

        self.person_counter = person_counter

        self.wavelength = 0.004996541

    def process(self, data):
        # Check if over threshold
        if np.amax(data["abs"]) > self.ampl_threshold:
            self.presence_array.append(data)
        # Else check if last was and evaluate and clear
        else:
            if len(self.presence_array) > 0:
                update_val = self.evaluate_detection(self.presence_array)
                self.presence_array.clear()
                return update_val

    def evaluate_detection(self, detection):
        ampl_dist = detection[-1]["com"] - detection[0]["com"]
        ampl_dir = 0
        if abs(ampl_dist) > self.distance_threshold:

            if DEBUG:
                print("ampl_dist: ", ampl_dist)
                input("Enter")

            if ampl_dist < 0:
                ampl_dir = -1
            else:
                ampl_dir = 1

            if ampl_dir == self.in_value:
                self.person_counter += 1
            else:
                self.person_counter -= 1
            return True
        else:
            return False



if __name__ == "__main__":
    main()