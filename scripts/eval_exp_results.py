#!/usr/bin/env python3

import os
import json
import subprocess
import argparse
import re

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


class EvalExpResults():
    def __init__(self,args):
        # self.results_dir = os.path.join(get_package_share_directory('ros_tracking'),args.results_dir)
        self.results_dir = os.path.join(Path(__file__).parent.parent, args.results_dir)
        self.split = args.split
        self.nuscenes_dir = args.nuscenes_dir
        self.dataset = args.dataset

    def evaluate_results(self):

        file_object = open(os.path.join(self.results_dir, "evaluated_results.txt"), "w")

        # Iterate through all .json files in results directory
        for (dirpath, dirnames, filenames) in os.walk(self.results_dir):

            for file in filenames:

                # Ignore non-result files
                if os.path.splitext(file)[-1] != '.json':
                    continue

                print("Evaluating experiment case: " + os.path.splitext(file)[0])
                file_object.write("EXPERIMENT CASE: " + os.path.splitext(file)[0] + "\n")

                # Execute nuScenes evaluation script
                proc = subprocess.run(["python3","scripts/evaluate.py", 
                                "--eval_set", self.split,
                                "--version", self.dataset,
                                "--dataroot", self.nuscenes_dir,
                                os.path.join(dirpath,file)], capture_output=True, text=True)

                # Capture the relevant parts of stdout and write to a text file
                final_results = re.search("(### Final results ###\n)(.*)(Per-class results:\n)(.*)(Aggregated results:\n)(.*)(Eval time:)(.*)(Rendering curves)",proc.stdout,re.MULTILINE|re.DOTALL)
                file_object.write(final_results.group(1))
                file_object.write(final_results.group(2))
                file_object.write(final_results.group(3))
                file_object.write(final_results.group(4))
                file_object.write(final_results.group(5))
                file_object.write(final_results.group(6))
                file_object.write("\n\n")

                print("Complete. \n")

            break # only visit root directory, not subdirectories
        file_object.close()


def main(args=None):
    parser = argparse.ArgumentParser()
    home_dir = Path.home()

    parser.add_argument(
        "--nuscenes-dir",
        "-n",
        default=home_dir / "nuscenes",
        help="Path to nuscenes data directory",
    )
    parser.add_argument(
        "--results-dir",
        "-r",
        default= "results",
        help="Relative path from ros_tracking to results.json files",
    )

    parser.add_argument(
        "--dataset",
        "-d",
        default="v1.0-mini",
        help="NuScenes dataset: v1.0-mini, v1.0-trainval, v1.0-test",
    )
    parser.add_argument(
        "--split",
        "-s",
        default="mini_val",
        help="NuScenes dataset split: mini_train, mini_val, train, val, test",
    )

    args = parser.parse_args()

    # Initialize converter object
    eval_exp_res = EvalExpResults(args)

    eval_exp_res.evaluate_results()


if __name__ == "__main__":
    main()