#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32

class DecisionTree:
    def __init__(self):
        rospy.init_node('decision_tree', anonymous=True)

        self.soc = 0  # short hand for State of Charge
        self.snow_counter = 0
        self.temperature = 0
        #self.wind_speed = 0

        # Defining Subscriber for weather forecast
        rospy.Subscriber('weather_forecast', String, self.weather_forecast_callback)
        rospy.Subscriber('/robot/battery', Int32, self.battery_callback)

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

    def battery_callback(self, data):
        self.soc = data.data

    def run_decision_tree(self):
        while not rospy.is_shutdown():
            if self.soc >= 60 and self.temperature <= 1 and self.snow_counter <= 3: #self.wind_speed <= 3
                rospy.loginfo("Clearing driveway")
                self.snow_counter = 0  # Reset snow counter
                self.launch_autonomous_mission()
            else:
                rospy.loginfo("Staying inside")
            self.rate.sleep()

    def launch_autonomous_mission(self):
        package = "snow_removal"  # Replace with your package name
        launch_file = "Autonomous_mission.launch"
        self.launcher.launch(package, launch_file)

if __name__ == '__main__':
    try:
        decision_tree = DecisionTree()
        decision_tree.run_decision_tree()
    except rospy.ROSInterruptException:
        pass
