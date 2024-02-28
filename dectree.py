#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

class DecisionTree:
    def __init__(self):
        rospy.init_node('decision_tree', anonymous=True)

        self.soc = 0  # short hand for State of Charge
        self.snow_counter = 0
        self.temperature = 0
        self.wind_speed = 0

        # Defining Subscribers
        rospy.Subscriber('soc', Float32, self.soc_callback)
        rospy.Subscriber('snow', Float32, self.snow_callback)
        rospy.Subscriber('temperature', Float32, self.temperature_callback)
        rospy.Subscriber('wind_speed', Float32, self.wind_speed_callback)

        self.rate = rospy.Rate(1 / 1800.0)  # scanning every 30 minutes

    def soc_callback(self, data):
        self.soc = data.data

    def snow_callback(self, data):
        snow_value = data.data
        self.snow_counter += snow_value # increment snow counter every time snow topic is updated 

    def temperature_callback(self, data):
        self.temperature = data.data

    def wind_speed_callback(self, data):
        self.wind_speed = data.data

    def run_decision_tree(self):
        while not rospy.is_shutdown():
            #if self.soc < 60:
                #rospy.loginfo("Staying in (SOC < 60)")
                if self.snow_counter > 3:
                    rospy.loginfo("Staying in (Snow counter > 3mm)")
                    if self.temperature > 1:
                        rospy.loginfo("Staying in (Temperature > 1°C)")
                        if self.wind_speed > 3:
                            rospy.loginfo("Staying in (Wind Speed > 3 km/h)")
                else:
                    rospy.loginfo("Clearing driveway")
                    self.snow_counter = 0  # Reset snow counter

                self.rate.sleep()

if __name__ == '__main__':
    try:
        decision_tree = DecisionTree()
        decision_tree.run_decision_tree()
    except rospy.ROSInterruptException:
        pass
