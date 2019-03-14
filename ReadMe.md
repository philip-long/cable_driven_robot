# Cable Driven Parallel Robot Package
## Package containing simple driver to commuinicate over TCP with B&R
## Collection of function useful for CDPR's
To launch a simple demo application (follow the order)


### Launch driver and simulated robot
`roslaunch br_motor_driver br_driver_sim.launch robot_ip:=127.0.0.1 port:=50001`

### Launch robot odemetry based on motor positions
` rosrun cable_rob tf_estim`

### Launch a simple GUI
`rosrun cable_rob tf_with_joint_gui.py`

### Launch rviz
` roslaunch cable_rob br_rviz.launch config:=true`
