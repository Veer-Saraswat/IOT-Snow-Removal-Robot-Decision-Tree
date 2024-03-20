#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import requests
import json
import time

class WeatherRobot:
    def __init__(self):
        rospy.init_node('weather_robot')
        # Defining ROS topics such as user_location and weather_forecast
        self.location_subscriber = rospy.Subscriber('/user_location', String, self.location_callback)
        self.weather_publisher = rospy.Publisher('/weather_forecast', String, queue_size=10)
        self.location = "Oshawa Ontario"  #None # holding users location
        self.api_key = 'a57e3487d4af4c39a9505707242501'  # Replace with API keyin quotes
        self.base_url = 'https://api.weatherapi.com/v1/current.json?'

    def location_callback(self, data):
        self.location = data.data

    def get_weather_forecast(self):
        if self.location: # Checking if users location is available 
            try:
                url = f"{self.base_url}key={self.api_key}&q={self.location}&aqi=no"
                response = requests.get(url)
                data = json.loads(response.text)
                if 'current' in data: # Checking if the current key exist in the data 
                    current = data['current'] # retrieves current data 
                    # retrieves the snow level, wind speed, if not available prints N/A
                    snow_mm = current.get('precip_mm', 'N/A') 
                    wind_speed = current.get('wind_kph', 'N/A')
                    temperature = current.get('temp_c', 'N/A')
                    # creating a string that contains snow level, wind speed and temperature and publishs it to the weather_forecast
                    forecast_data = f"Snow Level: {snow_mm} mm, Wind Speed: {wind_speed} kph, Temperature: {temperature} °C"
                    self.weather_publisher.publish(forecast_data)
                    rospy.loginfo("Weather forecast published successfully.")
                else:
                    rospy.logerr("Failed to fetch weather data.") # when weather cannot be retrieved 
            except Exception as e:
                rospy.logerr(f"Error fetching weather data: {str(e)}") # catching any exception during the request 
        else:
            rospy.logerr("No user location available.") # when users location is not available 

    def run(self):
        while not rospy.is_shutdown():
            self.get_weather_forecast()
            time.sleep(1800)  # Sleep for 30 minutes

if __name__ == '__main__':
    try:
        weather_robot = WeatherRobot()
        weather_robot.run()
    except rospy.ROSInterruptException:
        pass
