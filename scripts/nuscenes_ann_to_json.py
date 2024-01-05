#!/usr/bin/env python3

import argparse
import os
import json

from pathlib import Path

from nuscenes.nuscenes import NuScenes
import nuscenes.utils.splits as nuscenes_splits

def main(args=None):
    # Handle arguments
    parser = argparse.ArgumentParser()
    home_dir = Path.home()
    parser.add_argument(
        "--nuscenes-dir",
        "-n",
        default=home_dir / "nuscenes",
        help="Path to nuscenes data directory (input)",
    )
    parser.add_argument(
        "--ann-dir",
        "-a",
        default=home_dir / "nuscenes/annotations",
        help="Path to re-indexed .json annotation directory (output)",
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
        default="mini_train",
        help="NuScenes dataset split: mini_train, mini_val, train, val, test",
    )
    args = parser.parse_args()

    # Create nuscenes objects
    nusc = NuScenes(version=args.dataset, dataroot=args.nuscenes_dir, verbose=True)
    split = eval('nuscenes_splits.' + args.split)

    # If output directory doesn't exist, create it
    if not os.path.exists(args.ann_dir):
        os.mkdir(args.ann_dir)

    # Create annotation dictionary
    ann_dict = dict()

    # Iterate through scenes and add 
    for scene_name in split:

        # Lookup scene info from name
        for idx in range(len(nusc.scene)):
            if nusc.scene[idx]["name"]==scene_name:
                scene = nusc.scene[idx]

        # Initialize scene indices
        cur_sample = nusc.get("sample", scene["first_sample_token"])  
        epoch = 0

        # Re-index scene by object (annotation) / epoch / state, instead of epoch / object / state
        while cur_sample is not None:
            
            for annotation_id in cur_sample["anns"]:
                ann = nusc.get("sample_annotation", annotation_id)

                # If first time seeing the object, add it to the dictionary
                if ann['instance_token'] not in ann_dict.keys():
                    ann_dict[ann['instance_token']] = dict()
                    ann_dict[ann['instance_token']]['category'] = ann['category_name']
                    ann_dict[ann['instance_token']]['scene_name'] = scene_name
                    ann_dict[ann['instance_token']]['states'] = dict()

                # Now, add state information for this epoch
                ann_dict[ann['instance_token']]['states'][epoch] = dict()
                ann_dict[ann['instance_token']]['states'][epoch]['timestamp'] = cur_sample['timestamp']
                ann_dict[ann['instance_token']]['states'][epoch]['attributes'] = []
                
                for token in ann['attribute_tokens']:
                    ann_dict[ann['instance_token']]['states'][epoch]['attributes'].append(nusc.get("attribute", token)['name'])
                ann_dict[ann['instance_token']]['states'][epoch]['translation'] = ann['translation']
                ann_dict[ann['instance_token']]['states'][epoch]['rotation'] = ann['rotation'] 

            cur_sample = nusc.get("sample", cur_sample["next"]) if cur_sample.get("next") != "" else None
            epoch +=1

    # Write json file
    with open(os.path.join(args.ann_dir, args.split + '.json'),"w") as outfile:
        json.dump(ann_dict,outfile)
    
if __name__ == "__main__":
    main()