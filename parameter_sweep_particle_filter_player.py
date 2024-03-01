#!/usr/bin/env python3

import argparse
import subprocess

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Starts particle filter with flags')
    parser.add_argument('flag1', type=str, help='Flag1')
    args = parser.parse_args()
    
    subprocess.run('./bin/particle_filter', shell=True, cwd='/home/dev/cs393r_starter/', check=True)
    # print('Done playing bag!')
