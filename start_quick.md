### 3.1. A* + Minimum Snap :

- Quick start:
 # demo01
  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_minimum_jerk_astar.launch
  ```
  # demo02 
  ```bash
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  source devel/setup.bash
  rosrun test cmd_circle
  ```

