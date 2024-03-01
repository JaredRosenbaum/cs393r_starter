#!/usr/bin/env python3

import rospy
import time
import subprocess
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from amrl_msgs.msg import Localization2DMsg
import csv
import os
import numpy as np

class Trial ():
    def __init__(self, bags, runs, resampling_iteration_threshold, sigma_s, gamma, k3, k5) -> None:
        self.bags = bags
        self.runs = runs
        self.resampling_iteration_threshold = resampling_iteration_threshold
        self.sigma_s = sigma_s
        self.gamma = gamma
        self.k3 = k3
        self.k5 = k5
    
    def __str__(self):
        return f'Bags:\t\t{self.bags}\nRuns:\t\t{self.runs}\nResampling it:\t{self.resampling_iteration_threshold}\nSigma_s:\t{self.sigma_s}\nGamma:\t\t{self.gamma}\nk3:\t\t{self.k3}\nk5:\t\t{self.k5}'

class Sweeper():
    
    def __init__(self) -> None:
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.register_initialization)
        rospy.Subscriber('/reference_localization', Localization2DMsg, self.register_reference_localization)
        rospy.Subscriber('/localization', Localization2DMsg, self.register_estimated_localization)
        rospy.Service('/parameter_sweep', Trigger, self.run_schedule_service)
        
        self.initialized = False
        self.reference_list = []
        self.estimated_list = []
        
        # self.data_dir = '/home/steven/workspace/coursework/autonomous_robots/cs393r_starter/auxillary_ws/data/testing'
        self.data_dir = '/home/dev/cs393r_starter/auxillary_ws/data/testing'
    
    def run_schedule_service (self, req):
        trials = []
        
        bags = [
            '2020-04-01-17-08-55.bag',
            '2020-04-01-17-09-24.bag',
            '2020-04-01-17-14-01.bag',
            '2020-04-01-17-14-23.bag',
            '2020-04-01-17-20-04.bag',
            '2020-04-01-17-21-48.bag',
            '2020-04-01-17-22-44.bag',
            '2020-04-01-17-23-38.bag',
            '2020-04-01-17-25-18.bag',
            '2020-04-01-17-27-15.bag',
        ]
        n_plays = 1
        # resampling_iterations = [10, 20, 30]
        # sigmas = [0.3, 0.5, 0.8]
        # gammas = [0.05, 0.1, 0.5]
        # k3s = [0.5, 0.75, 1.0]
        # k5s = [0.2, 0.5, 0.8]
        
        # for resample_it in resampling_iterations:
        #     for sigma in sigmas:
        #         for gamma in gammas:
        #             for k3 in k3s:
        #                 for k5 in k5s:
        #                     trials.append(Trial(
        #                         bags,
        #                         n_plays,
        #                         resample_it,
        #                         sigma,
        #                         gamma,
        #                         k3,
        #                         k5,
        #                     ))
        
        trials.append(Trial(bags, n_plays, 10, 0.25, 0.1, 0.8, 0.3))
        
        print(f'Running {len(trials)} trials...')
        for i in range(len(trials)):
            self.run_trial(trials[i])
            print(f'Completed trial {i + 1} of {len(trials)} ({round(100 * (i + 1) / len(trials))}%)')
        print('Done with all trials!')
    
    def run_trial (self, trial:Trial):
        start_time = time.time()
        
        print(trial)
        
        data = [
            ['', 'Runs', 'Resampling It', 'Sigma_s', 'Gamma', 'k3', 'k5'],
            ['', trial.runs, trial.resampling_iteration_threshold, trial.sigma_s, trial.gamma, trial.k3, trial.k5],
        ]
        
        bag_mses = {}
        
        for bag in trial.bags:
            mses = []
        
            for _ in range(trial.runs):
                self.reference_list = []
                self.estimated_list = []
                particle_filter_process = subprocess.Popen([
                    'python', 'particle_filter_player.py',
                    # str(n_particles),
                    str(trial.resampling_iteration_threshold),
                    str(trial.sigma_s),
                    str(trial.gamma),
                    str(trial.k3),
                    str(trial.k5),
                ])
                time.sleep(1.0)
                play_bag_process = subprocess.Popen(['python', 'bag_player.py', bag], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                stdout, stderr = play_bag_process.communicate()
                
                # os.killpg(os.getpgid(particle_filter_process.pid), signal.SIGTERM)
                particle_filter_process.kill()
                subprocess.run('rosnode kill /particle_filter', shell=True, cwd='/home/dev/cs393r_starter/')
                time.sleep(3.0)
                
                self.initialized = False
                mses.append(self.compute_mse())
            
            # bag_mse = sum(bag_mses) / trial.runs
            bag_mses[bag] = sum(mses) / trial.runs
        
        average_mse = sum(bag_mses.values()) / len(trial.bags)
        
        filename = os.path.join(self.data_dir, str(average_mse) + str(time.time()) + '.csv')
        with open(filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            
            for bag in trial.bags:
                data.append(['', '', bag, bag_mses[bag]])
            
            csvwriter.writerows(data)
            
            elapsed_time = time.time() - start_time
            print('Trial took {:.2f} minutes to run.'.format(elapsed_time / 60))
            
            csvwriter.writerow(['Avg MSE', average_mse, '', elapsed_time, elapsed_time / 60])
    
    def reject_outliers(data, m = 2.):
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d/mdev if mdev else np.zeros(len(d))
        return data[s<m]
    
    def compute_mse (self):
        print(f'\tComputing MSE for {len(self.reference_list)} reference locations and {len(self.estimated_list)} estimated locations...')
        max_length = min(len(self.reference_list), len(self.estimated_list))
        if max_length < 1:
            return -1
        
        # TODO make this check for extreme outliers and reject them!!!
        mse = 0
        for i in range(max_length):
            mse += (self.reference_list[i].pose.x - self.estimated_list[i].pose.x)**2 + (self.reference_list[i].pose.y - self.estimated_list[i].pose.y)**2 + (0.01 * (self.reference_list[i].pose.theta - self.estimated_list[i].pose.theta))**2
        mse /= max_length
        
        print(f'MSE: {mse}')
        return mse
    
    def register_reference_localization (self, reference_localization):
        if self.initialized == False:
            return
        self.reference_list.append(reference_localization)
        # rospy.loginfo(rospy.get_caller_id() + f" Reference {len(self.reference_list)}")
    
    def register_estimated_localization (self, estimated_localization):
        if self.initialized == False:
            return
        self.estimated_list.append(estimated_localization)
        # rospy.loginfo(rospy.get_caller_id() + f" Estimation {len(self.estimated_list)}")
    
    def register_initialization (self, initial_pose):
        self.initialized = True
        # rospy.loginfo('\t' + rospy.get_caller_id() + ' starting to read topics!')
    
    def run (self):
        while not rospy.is_shutdown():
            # self.run_schedule()
            rospy.spin()
        # print("\n\nRUN END!\n\n")ss

if __name__ == '__main__':
    sweeper = Sweeper()
    sweeper.run()
