#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "ros.h"
#include "std_msgs/String.h"

#define URL "http://api.openweathermap.org/data/2.5/forecast?"
#define CurrentDataURL "https://api.openweathermap.org/data/2.5/weather?"
#define ApiKey  "0f9d90fba7e137f7a8fb3d82788df902"
#define ssid "VeersPhone"
#define password "Veer1234"
// Replace with your location Credentials
#define lat "43.8971"
#define lon "-78.8658"

#define TimeStamps 10  // number of timestamps to print

ros::NodeHandle nh;

std_msgs::String weather_msg;
ros::Publisher weather_pub("weather_data", &weather_msg);

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  nh.initNode();
  nh.advertise(weather_pub);
}

void publishWeatherData(const char* data) {
  weather_msg.data = data;
  weather_pub.publish(&weather_msg);
}

void print_current(void) {
  HTTPClient http;
  http.begin(String(CurrentDataURL) + "lat=" + String(lat) + "&lon=" + String(lon) + "&appid=" + String(ApiKey) + "&units=metric");
  int httpCode = http.GET();

  if (httpCode > 0) {
    String JSON_Data = http.getString();
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, JSON_Data);
    JsonObject obj = doc.as<JsonObject>();

    // Weather Extraction Logic/process 
    //2024-01-18 21:00:00

    // parse a JSON array


    if (!obj.containsKey("dt")) {
      Serial.println("Invalid timee");
    }

    long unix_time = obj["dt"].as<unsigned long>() + obj["timezone"].as<long>();
    struct tm* timeinfo;
    time_t epoch_time_as_time_t = unix_time;
    timeinfo = localtime(&epoch_time_as_time_t);

    Serial.println(localtime(&epoch_time_as_time_t));

    if (!obj["weather"][0].containsKey("main")) {
      Serial.println("Main doesnt exist");
    } else {
      String main_desc = obj["weather"][0]["main"].as<const char*>();
      Serial.println("  " + main_desc);
    }
    if (!obj["weather"][0].containsKey("description")) {
      Serial.println("Description not found");
    } else {
      String description = obj["weather"][0]["description"].as<const char*>();
      Serial.println("  " + description);
    }
    float temp;

    if (!obj["main"].containsKey("temp")) {
      Serial.println("Main temp not found");
    } else {
      float temp = obj["main"]["temp"].as<float>() - 0;
      Serial.println("    Temperature " + String(temp) + "°C");
    }

    if (!obj["main"].containsKey("feels_like")) {
      Serial.println("Main feels_like not found");
    } else {
      float feelsLike = obj["main"]["feels_like"].as<float>() - 0;
      Serial.println("    Feels Like " + String(feelsLike) + "°C");
    }
    // check Snow
    if (obj.containsKey("snow") == true) {
      float snow = obj["snow"]["1h"].as<float>();
      Serial.println("    Snow: " + String(snow) + "mm");
    } else
      Serial.println("    Snow: 0mm");

    // check rain exists
    if (obj.containsKey("rain") == true) {
      float rain = obj["rain"]["1h"].as<float>();
      Serial.println("    Rain: " + String(rain) + "mm");
    } else
      Serial.println("    Rain: 0mm");

    // check Humidity
    float humidity;
    if (!obj["main"].containsKey("humidity")) {
      Serial.println("Main Humidity not found");
    } else {
      int humidity = obj["main"]["humidity"].as<int>();
      Serial.println("    Humidity " + String(humidity) + "%");
    }

    
    // check clouds
    float wind_speed;
    if (!obj.containsKey("clouds")) {
      Serial.println("Clouds not found not found");
    } else {
      int cloud = obj["clouds"]["all"].as<int>();
      Serial.println("    Clouds: " + String(cloud) + "%");
    }
    if (!obj.containsKey("wind")) {
      Serial.println("Wind not found");
    } else {
      float wind_speed = obj["wind"]["speed"].as<float>();
      float wind_deg = obj["wind"]["deg"].as<float>();
      Serial.println("    Wind Speed: " + String(wind_speed) + "Km/h");
      Serial.println("    Wind Degree: " + String(wind_deg) + "°");
    }

    // Publish data to ROS
    String publish_data = "Temperature: " + String(temp) + "°C, Humidity: " + String(humidity) + "%, Wind Speed: " + String(wind_speed) + "Km/h";
    publishWeatherData(publish_data.c_str());
  }
  else {
    Serial.println("Error!");
  }

  http.end();
}

void loop() {
  nh.spinOnce();

  if (WiFi.status() == WL_CONNECTED) {
    print_current();
    delay(1200000);  // Wait for 20 minutes
  }
}
