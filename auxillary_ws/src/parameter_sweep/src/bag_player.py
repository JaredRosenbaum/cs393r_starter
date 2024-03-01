#!/usr/bin/env python3

import argparse
import subprocess

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Plays bag in known directory')
    parser.add_argument('bag', type=str, help='Name of bag')
    args = parser.parse_args()
    
    # subprocess.run('rosbag play ' + args.bag, shell=True, cwd='/home/dev/cs393r_starter/bags/2020/', check=True)
    subprocess.run('rosbag play ' + args.bag, shell=True, cwd='/home/dev/cs393r_starter/bags/2020/', check=True)
    # print('Done playing bag!')
