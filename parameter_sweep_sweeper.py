#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped
from amrl_msgs.msg import Localization2DMsg

class Sweeper():
    
    def __init__(self) -> None:
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.register_initialization)
        rospy.Subscriber("/reference_localization", Localization2DMsg, self.register_reference_localization)
        rospy.Subscriber("/localization", Localization2DMsg, self.register_estimated_localization)
        
        self.initialized = False
        self.reference_list = []
        self.estimated_list = []
    
    def run_schedule (self):
        particle_filter_process = subprocess.Popen(['python', 'parameter_sweep_particle_filter_player.py', 'dummy'])
        
        # for i in range(1):
            # self.run_trial()
            
        # play_bag_process = subprocess.Popen(['python', 'bag_player.py', '2020-04-01-17-23-38.bag'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # stdout, stderr = play_bag_process.communicate()
        
        # print('Play bag process finished with return code: ', play_bag_process.returncode)
        
        # self.initialized = False
        # print('Ready to compute the correspondences for data!')
        # self.compute_mse()
        # self.reference_list = []
        # self.estimated_list = []
    
    def run_trial (self):
        play_bag_process = subprocess.Popen(['python', 'parameter_sweep_bag_player.py', '2020-04-01-17-25-18.bag'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        particle_filter_process = subprocess.Popen(['python', 'parameter_sweep_particle_filter_player.py', 'dummy'])
        stdout, stderr = play_bag_process.communicate()
        
        print('Play bag process finished with return code: ', play_bag_process.returncode)
        
        particle_filter_process.kill()
        print('Stopped particle filter!')
        
        self.initialized = False
        print('Ready to compute the correspondences for data!')
        self.compute_mse()
        self.reference_list = []
        self.estimated_list = []
    
    def compute_mse (self):
        max_length = min(len(self.reference_list), len(self.estimated_list))
        
        mse = 0
        for i in range(max_length):
            mse += (self.reference_list[i].pose.x - self.estimated_list[i].pose.x)**2 + (self.reference_list[i].pose.y - self.estimated_list[i].pose.y)**2
        mse /= max_length
        
        print(f'MSE: {mse}')
    
    def register_reference_localization (self, reference_localization):
        if not self.initialized:
            return
        self.reference_list.append(reference_localization)
        # rospy.loginfo(rospy.get_caller_id() + f" Reference {len(self.reference_list)}")
    
    def register_estimated_localization (self, estimated_localization):
        if not self.initialized:
            return
        self.estimated_list.append(estimated_localization)
        # rospy.loginfo(rospy.get_caller_id() + f" Estimation {len(self.estimated_list)}")
    
    def register_initialization (self, initial_pose):
        rospy.loginfo(rospy.get_caller_id() + " Starting to read topics!")
        self.initialized = True
    
    def run (self):
        while not rospy.is_shutdown():
            self.run_schedule()
            rospy.spin()
            # print(f'{len(self.reference_list)}, {len(self.estimated_list)}')

if __name__ == '__main__':
    sweeper = Sweeper()
    sweeper.run()
