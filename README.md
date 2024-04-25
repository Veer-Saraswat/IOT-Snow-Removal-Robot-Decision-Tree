- These files are to be used for the decision making process of the Autonomous Snow removal robot
- weather API file grabs information based on the users location input from the mobile app, and grabs information such as snow level, temperature, and wind speed
- the weather API file passes the information gathered into the decision tree to determine if the robot should go out and clear the driveway, this process is forecasted every 30 mins.
- In additon the battery monitering system is also passing real time information on the robots battery percetage into the decision tree from an Arduino Api 
- If the conditions are met, the decision tree will launch a file in which it contains the saved map of the users driveway, and begin to clear the snow

- As of now the weather api file has to be launched first and then the decision tree file can then be activated. Ideally these files should activate once the user has saved their driveway map
- These files are to be in the same ros package in order for it to properly work
- refer to https://github.com/Maaz-Sidd/IoT-snow-Removal-ROS , for the entire Autonomus snow removal ROS files, including the front end aspect of the mobile application

- The weather API will require an API key that is to be typed in from whichever website the user uses, once that is set, the information can be retrieved consistently 
