'''
Author: Rajat Kumar Jenamani
Runs a server-side (run on NUC) bulldog to ensure robot is not in a state of emergency stop from the experimentor 
emergency stop button. Can also listen to an additional emergency stop button provided to the user. 
'''

import rospy
import numpy as np
import time
from enum import Enum
import queue
import signal
import sys

import threading
import time
import numpy as np
import argparse

import rospy
from std_msgs.msg import Bool

from kinova_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY

USER_ESTOP_FREQUENCY_THRESHOLD = 50 # expected is 60 Hz
EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD = 50 # expected is 60 Hz

BULLDOG_RUN_FREQUENCY = 1000

from feeding_deployment.safety.utils import PeekableQueue, AnomalyStatus

class BullDog:
    def __init__(self, experimentor_estop_topic, use_user_estop, user_estop_topic):
        
        self.use_user_estop = use_user_estop
        print("BullDog awakening...")

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()
        

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        queue_size = 1000
        self.experimentor_emergency_stop_sub = rospy.Subscriber(experimentor_estop_topic, Bool, self.experimentorEmergencyStopCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.experimentor_emergency_stop_timestamps = PeekableQueue()
        self.experimentor_emergency_stop_pressed = False

        if self.use_user_estop:
            self.user_emergency_stop_sub = rospy.Subscriber(user_estop_topic, Bool, self.userEmergencyStopCallback, queue_size = queue_size, buff_size = 65536*queue_size)
            self.user_emergency_stop_timestamps = PeekableQueue()
            self.user_emergency_stop_pressed = False

        self.bulldog_status_pub = rospy.Publisher('/bulldog_status', Bool, queue_size=1)

        self.second_counter = 0
        time.sleep(1.0)
        print("BullDog is guarding the robot... ▽ ᴥ ▽")

    def userEmergencyStopCallback(self, msg):

        self.user_emergency_stop_timestamps.put(time.time())
        if msg.data:
            self.user_emergency_stop_pressed = True

    def experimentorEmergencyStopCallback(self, msg):

        self.experimentor_emergency_stop_timestamps.put(time.time())
        if msg.data:
            self.experimentor_emergency_stop_pressed = True

    def check_status(self):
        self.second_counter += 1
        self._arm_interface.is_alive()
        anomaly = AnomalyStatus.NO_ANOMALY
        start_time = time.time()
        frequencies = []

        if self.use_user_estop:
            estop_frequency_data = [(self.user_emergency_stop_timestamps, USER_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.USER_ESTOP_FREQUENCY),
                            (self.experimentor_emergency_stop_timestamps, EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.EXPERIMENTOR_ESTOP_FREQUENCY)]
        else:
            estop_frequency_data = [(self.experimentor_emergency_stop_timestamps, EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.EXPERIMENTOR_ESTOP_FREQUENCY)]

        for _queue, _threshold, _anomaly in estop_frequency_data:
            while _queue.peek() < start_time - 1.0:
                _queue.get()
            queue_size = _queue.qsize()
            if queue_size < _threshold:
                print(f"Frequency: {queue_size} for {_anomaly}")
                rospy.loginfo(f"Frequency: {queue_size} for {_anomaly}")
                anomaly = _anomaly
                break   
            frequencies.append(queue_size)

        if self.second_counter == BULLDOG_RUN_FREQUENCY:
            print("Bulldog running at expected frequency.")
            if self.use_user_estop:
                print(f"Frequencies: User EStop: {frequencies[0]}, Experimentor EStop: {frequencies[1]}")
            else:
                print(f"Frequencies: Experimentor EStop: {frequencies[0]}")
            self.second_counter = 0

        if self.use_user_estop:
            estop_data = [(self.user_emergency_stop_pressed, AnomalyStatus.USER_ESTOP_PRESSED),
                        (self.experimentor_emergency_stop_pressed, AnomalyStatus.EXPERIMENTOR_ESTOP_PRESSED)]
        else:
            estop_data = [(self.experimentor_emergency_stop_pressed, AnomalyStatus.EXPERIMENTOR_ESTOP_PRESSED)]

        for _unexpected, _anomaly in estop_data:
            if _unexpected:
                print(f"Unexpected: {_anomaly}")
                rospy.loginfo(f"Unexpected: {_anomaly}")
                anomaly = _anomaly
                break

        if anomaly != AnomalyStatus.NO_ANOMALY:
            print(f"AnomalyStatus detected: {anomaly}")
            rospy.loginfo(f"AnomalyStatus detected: {anomaly}")
            self._arm_interface.emergency_stop() 

        self.bulldog_status_pub.publish(Bool(data=anomaly == AnomalyStatus.NO_ANOMALY))
        return anomaly
    
    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            status = self.check_status()
            if status != AnomalyStatus.NO_ANOMALY:
                print(f"AnomalyStatus detected: {status}")
                rospy.loginfo(f"AnomalyStatus detected: {status}")
                break
            end_time = time.time()
            # print(f"Time taken: {end_time - start_time}")
            time.sleep(max(0, 1.0/BULLDOG_RUN_FREQUENCY - (end_time - start_time)))

if __name__ == '__main__':

    args = argparse.ArgumentParser()
    
    # estop_topic with default estop
    args.add_argument("--experimentor_estop_topic", type=str, default="/estop", help="Experimentor emergency stop topic")
    args.add_argument("--use_user_estop", type=bool, default=False, help="Use another estop that is provided to the user")
    args.add_argument("--user_estop_topic", type=str, default="/experimentor_estop", help="Experimentor emergency stop topic")
    args = args.parse_args()

    rospy.init_node('BullDog', anonymous=True)
    bulldog = BullDog(args.experimentor_estop_topic, args.use_user_estop, args.user_estop_topic)
    
    bulldog.run()
    