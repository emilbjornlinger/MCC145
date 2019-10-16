import numpy as np
import pickle
from time import sleep, time

from acconeer_utils.clients import UARTClient, SPIClient, SocketClient
from acconeer_utils.clients import configs
from acconeer_utils import example_utils
from acconeer_utils.pg_process import PGProcess, PGProccessDiedException


def main():
    #load_data()
    sensor_read()


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

    session_info = client.setup_session(sensor_config) # Needed for customProcess

    client.start_streaming()

    interrupt_handler = example_utils.ExampleInterruptHandler()
    print("Press Ctrl-C to end session")

    processor = PhaseTrackingProcessor(sensor_config.sweep_rate)
    while not interrupt_handler.got_signal:
        info, sweep = client.get_next()
        plot_data = processor.process(sweep)

        if plot_data is not None:
            try:
                print(np.amax(plot_data["abs"]))

                # This is where the processing comes in
                # Call customProcess

                # Create a vector that is of correct length, then look at the history, call processVEctor
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


def load_data():
    # get data
    file_name = "sample.pkl"
    with open(file_name, "rb") as infile:
        data = pickle.load(infile)
    file_name_meta = "metasample.pkl"
    with open(file_name_meta, "rb") as infile:
        session_info = pickle.load(infile)

    # Extract metadata
    range_start = session_info["actual_range_start"]
    range_length = session_info["actual_range_length"]
    sweep_rate = session_info["frequency"]
    data_length = session_info["data_length"]

    processor = PhaseTrackingProcessor(sweep_rate)

    # Set sleep behaviour
    use_sleep = False
    ans = input("Use sleep: [Y/n]")
    if ans == "Y" or ans == "y":
        use_sleep = True
    per_time = 1 / sweep_rate

    # Instantiate customProcess
    custom_processor = customProcess(range_length, range_start, 0.2, data_length, -1)

    for i in data:
        start_time = time()
        plot_data = processor.process(i)

        if plot_data is not None:
            abs = plot_data["abs"]
            maxVal = np.amax(abs)
            print("MaxValue:", maxVal)

            if custom_processor.evaluate_detection(plot_data) == True:
                # look at personcounter and convey information
                x = 1

        # Handle sleep time
        if use_sleep:
            end_time = time()
            sleep_time = per_time - (end_time - start_time)
            if sleep_time > 0:
                sleep(sleep_time)


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
    def __init__(self, range_interval, range_start, distance_threshold, data_length, in_value, person_counter=0):
        self.presence_array = []
        self.range_interval = range_interval
        self.range_start = range_start
        self.distance_threshold = distance_threshold
        self.data_length = data_length
        self.in_value = in_value

        self.person_counter = person_counter

    def process(self, data):
        # Check if over threshold
        if np.amax(data["abs"]) > self.distance_threshold:  # This needs finetuning, create a delay effect
            self.presence_array.append(data)
        # Else check if last was and evaluate and clear
        else:
            if len(self.presence_array) > 0:
                update_val = self.evaluate_detection(self.presence_array)
                self.presence_array.clear()
                return update_val

    def evaluate_detection(self, detection):
        ampl_dist = detection(-1)["com"] - detection(0)["com"]
        ampl_dir = 0
        phase_dir = 0
        phase_dist = 0
        if abs(ampl_dist) > self.distance_threshold:
            if ampl_dist < 0:
                ampl_dir = -1
            else:
                ampl_dir = 1
            phase_val = self.evaluate_phase(detection)
            phase_dir = phase_val(0)
            phase_dist = phase_val(1)
            if phase_dir == ampl_dir and phase_dist > self.distance_threshold:
                if phase_dir == self.in_value:
                    self.person_counter += 1
                    return True
                else:  # ampl_dir can't be 0, so phase_dir != 0, so no need to check for 0
                    self.person_counter -= 1
                    return True
        else:
            return False

    def evaluate_phase(self, detection):

        # loop through detection
        angle_arr = []
        for i in detection: # Is it data length we want to look at? Gives num points in the array?
            # get data index from com, not exact? Shouldn't matter since there will still be reliable phase at com?
            com = i["com"]
            fract_into_data_length = ((com - self.range_start) / self.range_interval) * self.data_length
            phase_index = round(fract_into_data_length)
            angle_arr.append(detection["arg"](phase_index)) #  Typecast instead?

        # Unwrap to get real phase
        angle_arr = np.unwrap(angle_arr)

        # Calculate total angle shift
        #total_shift =

        # Get distance and then direction from this
        #phase_dist =
        #phase_dir =


        # return list with dir and dist
        val = []
        val.append(-1)
        val.append(0.2)
        return val



if __name__ == "__main__":
    main()
