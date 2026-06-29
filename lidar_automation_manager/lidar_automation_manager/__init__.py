# Notes on what I had to do manually for all this

# 1. i had to manually set the ip address of each lidar
# 2. i had to run the ros driver myself
# 3. i had to verify via foxglove that the environment was fine and everything looked fine/correct
# 4. i had to choose the parameter configurations for the lidar
# 5. i had to definen their ranges, boolean, etc.
# 6. i had to manually run through all test cases and configurations
# 7. i had to kill the ros driver process, create new bags/directories and record rodbags, save bags, etc.
# 8. i had to manually test the angles at which to set the extremes at



# things i want to automate/use

# 1. i want to have a yaml file where i can list parameters of known interest and then list their ranges, values, etc. for each lidar
# 2. i want to keep each driver as an external repo in repos.yaml and then have the driver yaml files stored somewhere
# 3. i want to have a mode where i can calibrate the lidar first
# 4. once calibrated, i want a user input interface where users can input the 2 or however many angles they want to test against
# 5. i want lidar to record metrics at base angle (0 degrees first) and then go all the way to the left inwards and then all the way to the right inwards
# 6. i want to use bag recorder to record the rosbags
# 7. i want to use polymath-data-cli at the end to upload/push all bags to nexus
# 8. no weather conditions yet
# 9. i want the folder structure to be as follows:

# 1.


# manager flow


# 1. Put info about each lidar like it's package, executable name, etc. into lidar yaml file
# 2. upload that via polysetup or something for now (later on ansible,  i want to have a button on polylgot where you can click it and lidar test bench starts running)
# 2.1. polysetup then builds the packages in the background
# 3. user clicks begin -> starts lidar test bench
# 4. test case runner first createsthe folders according to a schema and then calls a test generator file
# 5. test case generator file runs (runs through each test case written and then generates test case runs (base, angle, parameterconfig) as key,value pairs)
# 6. main node gets the dict back
# 7. loop through each key in dict
# 8. as soon as a parameter paranet key is detected, implement a state machine to switch to a mode where a module finds the param file, edits it, then relaunches the specific node with it
# 9. trigger bag recorder somewhere in the background
# 10. record for some amount of seconds defined in yaml file
# 11. save the bag to the specified folder
# 12. repeat cycle
# 13. after all parameter files have been saved, ask user if they want to change anything on lidar, user does it and then enters name of the param + value -> when done, user just clicks enter
# 14. on angle time, the user resets everything to default value, and then sends message to arduino shit and then records and savesa
# 15. once everything is done, send /start_evaluation request



# package overview/architecture:

# 1. divide shit into nodes folder and tools folder
# 2. have a wrapper somewhere for bag recorder and polymath-data-cli

# standard SOP for user testing new lidar in a new environment:
