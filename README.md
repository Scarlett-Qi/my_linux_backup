# Build
在编译代码前需要source以下路径。
```
source ~/ros2_workspace/install/setup.bash
source ~/mima_intf_ws/install/setup.bash
cd ~/workspace
colcon build
```
# Run Code
在工控机根目录上`source setup_env.sh`脚本，然后运行一下命令启动，即可。
```
ros2 launch gr_planning forkliftPlanningPath.launch.py
ros2 run gr_planning forklift_action_server
```
