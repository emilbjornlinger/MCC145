# Create simple test, migt not work

from ProcessData import load_data
import json
import pandas as pd

DEBUG = False
DEBUG_PRINT = False
AMPL_THRESHOLD = [0.1, 0.04, 0.02, 0.01, 0.005]
DIST_THRESHOLD = [0.5, 0.2, 0.1, 0.05]

def main():

    #create_data()
    #create_xls()

    #Load file with data
    with open("dataSet.json", "r") as f:
        set_object = json.load(f)

    # Create output structure
    output_dict = set_object


    # Loop through each fileset
    counter = 0
    for i in set_object:
        # Get set info
        file_name = set_object[i][0]["file"]
        meta_file_name = set_object[i][0]["metafile"]
        dir = set_object[i][0]["dir"]

        # Perform run of program
        for j in AMPL_THRESHOLD:
            for k in DIST_THRESHOLD:
                person_counter = load_data(False, DEBUG, DEBUG_PRINT, j, k, file_name, meta_file_name)

                # Evaluate set
                success = False
                if person_counter == dir:
                    success = True
                output_dict[i][0][str(j) + ", " + str(k)] = success

    # Create output
    with open('dataResult.json', 'w', encoding='utf-8') as f:
        json.dump(output_dict, f, ensure_ascii=False, indent=4)


def create_data():

    x = {
        "set1": [{"file": "30mi.pkl", "metafile": "meta30mi.pkl", "dir": 1}],
        "set2": [{"file": "30mu.pkl", "metafile": "meta30mu.pkl", "dir": -1}],
        "set3": [{"file": "30si.pkl", "metafile": "meta30si.pkl", "dir": 1}],
        "set4": [{"file": "30su.pkl", "metafile": "meta30su.pkl", "dir": -1}],
        "set5": [{"file": "45mi.pkl", "metafile": "meta45mi.pkl", "dir": 1}],
        "set6": [{"file": "45mu.pkl", "metafile": "meta45mu.pkl", "dir": -1}],
        "set7": [{"file": "45si.pkl", "metafile": "meta45si.pkl", "dir": 1}],
        "set8": [{"file": "45su.pkl", "metafile": "meta45su.pkl", "dir": -1}],
        "set9": [{"file": "60mi.pkl", "metafile": "meta60mi.pkl", "dir": 1}],
        "set10": [{"file": "60mu.pkl", "metafile": "meta60mu.pkl", "dir": -1}],
        "set11": [{"file": "60si.pkl", "metafile": "meta60si.pkl", "dir": 1}],
        "set12": [{"file": "60su.pkl", "metafile": "meta60su.pkl", "dir": -1}]
    }

    with open('dataSet.json', 'w', encoding='utf-8') as f:
        json.dump(x, f, ensure_ascii=False, indent=4)

def create_xls():
    # Load file with data
    with open("dataResult.json", "r") as f:
        data_object = json.load(f)

    json_string = json.dumps(data_object)

    df = pd.read_json(json_string)
    df.to_excel("output.xls", index=False)

if __name__ == "__main__":
    main()