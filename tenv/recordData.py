import pickle

from acconeer_utils.clients import UARTClient, SPIClient, SocketClient
from acconeer_utils.clients import configs
from acconeer_utils import example_utils

from time import sleep, time


def main():

    recordData()


def recordData():
    args = example_utils.ExampleArgumentParser().parse_args()

    example_utils.config_logging(args)

    # Pick client depending on whether socket, SPI, or UART is used
    if args.socket_addr:
        client = SocketClient(args.socket_addr)
    elif args.spi:
        client = SPIClient()
    else:
        port = args.serial_port or example_utils.autodetect_serial_port()
        client = UARTClient(port)

    config = configs.IQServiceConfig()

    config.sensor = args.sensors
    config.range_interval = [0.3, 0.8]
    config.sweep_rate = int(input("Sweep rate (Hz): "))
    config.gain = 0.7

    client.connect()
    session_info = client.setup_session(config)
    dataLength = session_info["data_length"]
    print(dataLength)
    client.start_streaming()

    # Array of sweeps
    data = []
    numSweeps = int(input("Set number of sweeps: "))

    #Control time
    ses_time_start = time()

    #Used for sleep time
    per_time = 1/config.sweep_rate

    # Loop to get data from sensor
    for i in range(numSweeps):
        start_time = time()

        # Get data
        sweep_info, sweep_data = client.get_next()

        # Append to list
        data.append(sweep_data)

        # Sleep time according to frequency
        end_time = time()
        sleep_time = per_time - (end_time - start_time)
        if sleep_time > 0:
            sleep(sleep_time)

    #Control of time
    ses_time = time() - ses_time_start
    print("ses_time: ", ses_time)

    object = input("Object: (W/D) ")
    direction = input("direction (I/O, only for door: ")

    # Save to file
    with open("data" + str(config.sweep_rate) + str(numSweeps) + object + direction + ".pkl", "wb") as outfile:
        pickle.dump(data, outfile, pickle.HIGHEST_PROTOCOL)
    with open("metadata" + str(config.sweep_rate) + str(numSweeps) + object + direction + ".pkl", "wb") as outfile:
        pickle.dump(session_info, outfile, pickle.HIGHEST_PROTOCOL)


    client.stop_streaming()
    client.disconnect()

if __name__ == "__main__":
    main()

