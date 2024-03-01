#!/usr/bin/env python3

import rospy
import time
import subprocess
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from amrl_msgs.msg import Localization2DMsg
import csv
import os

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
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 1.0, 0.8))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.8, 0.5))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.5, 0.3))
        
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
        trials.append(Trial(bags, 2, 10, 0.25, 0.1, 0.8, 0.3))
        # runs.append(Trial('2020-04-01-17-23-38.bag', 3, 10, 0.5, 0.1, 0.8, 0.3))
        # runs.append(Trial('2020-04-01-17-23-38.bag', 3, 10, 0.75, 0.1, 0.8, 0.3))

        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.25, 0.2, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.5, 0.2, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.2, 0.8, 0.2))
        
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.25, 0.1, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.5, 0.1, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.1, 0.8, 0.2))
        
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.25, 0.05, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.5, 0.05, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.05, 0.8, 0.2))
        
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.25, 0.01, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.5, 0.01, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.75, 0.01, 0.8, 0.2))
        
        # runs.append(Run('2020-04-01-17-23-38.bag', 10, 0.5, 0.05, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 20, 0.5, 0.05, 0.38, 0.35))
        # runs.append(Run('2020-04-01-17-23-38.bag', 30, 0.5, 0.05, 0.8, 0.5))
        
        # runs.append(Run('2020-04-01-17-23-38.bag', 10, 0.5, 0.05, 0.8, 0.2))
        # runs.append(Run('2020-04-01-17-23-38.bag', 10, 0.5, 0.05, 0.38, 0.35))
        # runs.append(Run('2020-04-01-17-23-38.bag', 10, 0.5, 0.05, 0.8, 0.5))
        for trial in trials:
            self.run_trial(trial)
        print('Done with all trials!')
    
    def run_trial (self, trial:Trial):
        print(trial)
        
        data = [
            ['', 'Bag', 'Runs', 'Resampling It', 'Sigma_s', 'Gamma', 'k3', 'k5'],
            ['', trial.bag, trial.runs, trial.resampling_iteration_threshold, trial.sigma_s, trial.gamma, trial.k3, trial.k5],
        ]
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
            play_bag_process = subprocess.Popen(['python', 'bag_player.py', trial.bag], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = play_bag_process.communicate()
            
            # os.killpg(os.getpgid(particle_filter_process.pid), signal.SIGTERM)
            particle_filter_process.kill()
            subprocess.run('rosnode kill /particle_filter', shell=True, cwd='/home/dev/cs393r_starter/')
            time.sleep(3.0)
            
            self.initialized = False
            mses.append(self.compute_mse())
        
        average_mse = sum(mses) / trial.runs
        filename = os.path.join(self.data_dir, str(average_mse) + str(time.time()) + '.csv')
        with open(filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(data)
            mse_row = [average_mse]
            for mse in mses:
                mse_row.append(mse)
            csvwriter.writerow(mse_row)
    
    def compute_mse (self):
        print(f'\tComputing MSE for {len(self.reference_list)} reference locations and {len(self.estimated_list)} estimated locations...')
        max_length = min(len(self.reference_list), len(self.estimated_list))
        if max_length < 1:
            return -1
        
        mse = 0
        for i in range(max_length):
            mse += (self.reference_list[i].pose.x - self.estimated_list[i].pose.x)**2 + (self.reference_list[i].pose.y - self.estimated_list[i].pose.y)**2 + (self.reference_list[i].pose.theta - self.estimated_list[i].pose.theta)**2
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
