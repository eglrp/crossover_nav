# crossover_nav
How to run dataset

roscore
rosparam set use_sim_time true
rosbag play --clock /media/fx/34AC6825AC67E03A/frontin_backout.bag
roslaunch crossover_nav msf_optical_flow_vel.launch


to visualize path and map

roslaunch mav_visualization pure_visual_quad.launch
roslaunch mav_visualization rviz.launch
rqt_multiplot