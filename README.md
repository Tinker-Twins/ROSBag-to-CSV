# ROSBag to CSV

A ROS-based PyQt GUI tool to conveniently extract data from bag file(s) to CSV file(s).

## Setup:
```bash
$ cd ~/catkin_ws/src  
$ git clone https://github.com/Tinker-Twins/ROSBag-to-CSV.git
$ cd ~/catkin_ws && rosdep install -r --ignore-src --from-paths src
$ catkin_make
```

## Usage:

1. Execute the `rosbag_to_csv` node:
   
    a. `rosrun` (recommended)
    ```bash
    $ # New terminal window/tab
    $ roscore
    $ # New terminal window/tab
    $ rosrun rosbag_to_csv rosbag_to_csv.py
    ```
    
    b. `roslaunch` (alternative)
    ```bash
    $ # New terminal window/tab
    $ roslaunch rosbag_to_csv rosbag_to_csv.launch
    ```

3. Select a single bag file or multiple bag files using the GUI popup.

4. Select the topic(s) of interest that should be extracted to the CSV file(s) using the GUI popup.

5. Wait for a few seconds until the conversion takes place.

## Notes:

The CSV file(s) will be available at the path pointed in terminal (e.g. `$HOME`) or at `$HOME/.ros`.
