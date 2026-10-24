Membuat File Baru: cd ~/ros2_ws colcon build source ~/.bashrc

Membuka Gazebo Harmonic: ros2 launch robin_bringup my_robot_gazebo.launch.py

Menjalan topik yang bisa digunakan: source /opt/ros/jazzy/setup.bash
source /home/akmal-muzhaffar/ros2_ws/install/setup.bash
ros2 topic info /cmd_vel


Menjalankan Running Kontrol Robot Simulator: ros2 run simple_mover mover_node
