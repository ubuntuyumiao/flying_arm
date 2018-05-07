gnome-terminal -x bash -c "rqt_plot /ymdefined/imu/erlue/data[0]:data[1]:data[2] "  
  
gnome-terminal -x bash -c "rqt_plot /mavros/imu/data/linear_acceleration" 
 
gnome-terminal -x bash -c "rqt_plot /mavros/imu/data/angular_velocity" 