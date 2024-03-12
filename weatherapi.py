#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import json
from datetime import datetime, timedelta

class SnowRemovalRobot:
    def __init__(self):
        rospy.init_node('snow_removal_robot')

        # ROS subscribers
        rospy.Subscriber('user_location', String, self.location_callback)

        # ROS publishers
        self.snow_pub = rospy.Publisher('snow_level', String, queue_size=10)
        self.wind_pub = rospy.Publisher('wind_speed', String, queue_size=10)
        self.temp_pub = rospy.Publisher('temperature', String, queue_size=10)

        # Variables to store user location
        self.user_location = None

    def location_callback(self, data):
        self.user_location = data.data

    def get_weather_info(self):
        if self.user_location:
            try:
                api_key =  a57e3487d4af4c39a9505707242501  # Replace with WeatherAPI key
                url = f"https://api.weatherapi.com/v1/current.json?key={api_key}&q={self.user_location}"
                response = requests.get(url)
                data = json.loads(response.text)
                snow_level = data['current']['snow_mm']
                wind_speed = data['current']['wind_kmh']
                temperature = data['current']['temp_c']

                self.snow_pub.publish(str(snow_level))
                self.wind_pub.publish(str(wind_speed))
                self.temp_pub.publish(str(temperature))
            except Exception as e:
                rospy.logerr(f"Failed to fetch weather information: {str(e)}")

    def run(self):
        rate = rospy.Rate(1/1800)  # 1800 seconds = 30 minutes
        while not rospy.is_shutdown():
            self.get_weather_info()
            rate.sleep()

if __name__ == '__main__':
    try:
        snow_removal_robot = SnowRemovalRobot()
        snow_removal_robot.run()
    except rospy.ROSInterruptException:
        pass
