#!/usr/bin/env python3

import argparse
import subprocess

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Starts particle filter with flags')
    # parser.add_argument('n_particles', type=str, help='n_particles')
    parser.add_argument('resampling_iteration_threshold', type=str, help='resampling_iteration_threshold')
    parser.add_argument('sigma_s', type=str, help='sigma_s')
    parser.add_argument('gamma', type=str, help='gamma')
    parser.add_argument('d_long', type=str, help='d_long')
    parser.add_argument('d_short', type=str, help='d_short')
    parser.add_argument('k2', type=str, help='k2')
    parser.add_argument('k3', type=str, help='k3')
    parser.add_argument('k5', type=str, help='k5')
    parser.add_argument('k6', type=str, help='k6')

    args = parser.parse_args()
    
    # subprocess.run('./bin/particle_filter', shell=True, cwd='/home/dev/cs393r_starter/', check=True)
    # subprocess.run('./bin/particle_filter', shell=True, cwd='/home/dev/cs393r_starter/', check=True)
    subprocess.run('./bin/particle_filter' 
                + ' -resampling_iteration_threshold ' + args.resampling_iteration_threshold
                + ' -sigma_s ' + args.sigma_s
                + ' -gamma ' + args.gamma
                + ' -d_long ' + args.d_long
                + ' -d_short ' + args.d_short
                + ' -k2 ' + args.k2
                + ' -k3 ' + args.k3
                + ' -k5 ' + args.k5
                + ' -k6 ' + args.k6
                , shell=True, cwd='/home/dev/cs393r_starter/', check=False)
    # print('Done playing bag!')
