#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class DecisionTree:
    def __init__(self):
        rospy.init_node('decision_tree', anonymous=True)

        self.soc = 0  # short hand for State of Charge
        self.snow_counter = 0
        self.temperature = 0
        self.wind_speed = 0

        # Defining Subscriber for weather forecast
        rospy.Subscriber('weather_forecast', String, self.weather_forecast_callback)

        self.rate = rospy.Rate(1 / 1800.0)  # scanning every 30 minutes

    def weather_forecast_callback(self, data):
        weather_data = data.data.split(', ')
        for item in weather_data:
            key, value = item.split(': ')
            if key == 'Snow Level':
                self.snow_counter += float(value.replace(' mm', ''))
            elif key == 'Temperature':
                self.temperature = float(value.replace(' °C', ''))
            elif key == 'Wind Speed':
                self.wind_speed = float(value.replace(' kph', ''))

    def run_decision_tree(self):
        while not rospy.is_shutdown():
            if self.soc < 60:
                rospy.loginfo("Staying in (SOC < 60)")
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
