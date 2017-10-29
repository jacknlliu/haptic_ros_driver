# haptic_ros_driver
haptic device Omega 3 ros driver

# Usage
use `catkin_make` build this package, then run

```
rosrun haptic_ros_driver haptic_ros_driver
```

# Published topic
- `/haptic/position`

type:  
`geometry_msgs::Vector3Stamped`

- `/haptic/button_state`

type:  
`std_msgs::Int8MultiArray`

# Suscribed topic

- `/haptic/force`

type:  

`geometry_msgs::Vector3`


The motivation of this project is that I met a problem [force dimension haptic device SDK incompatible with moveit](https://github.com/jacknlliu/moveit_unit_test/issues/1).
