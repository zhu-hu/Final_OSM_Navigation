# nmea_navsat_driver


## Usage:

roslaunch nmea_navsat_driver sn_gps.launch

## Explanation:

The package is modified by **Tianyi Li**, as she wants to output the information of #HEADINGA and #BESTVELA

After her modification, the package can output **/fix**, **/heading** and **/vel**

- **fix**: lat, lon, height, quality, status

- **heading**: east-0, (-pi, pi], counterclockwise, unit: rad

- **vel**: x-east, y-north, unit: m/s


## Origin API

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver
