# Main steps


## Change the ``franka_ros2`` packages

1. ``git clone`` and ``colcon build`` the ``franka_ros2`` packages (detail steps please follow its ``README.md``)

    ```
    https://github.com/frankarobotics/franka_ros2.git
    ```

2. change the cartesian_pose_example_controller and its headers
- put the ``cartesian_pose_example_controller.cpp`` to ``/franka_ros2_ws/src/franka_example_controllers/src``
- put its headers to the ``/franka_ros2_ws/src/franka_example_controllers/include/franka_example_controllers``


3. rebuild and source
    ```
    cd franka_ros2_ws/
    ```

    ```
    colcon build 
    ```

    ```
    source install/setup.bash
    ```



## Terminal 1

1. go to your own ```franka_ros2``` workspace

    ```
    cd franka_ros2_ws/
    ```

2. source your ```franka_ros2``` workspace

    ```
    source install/setup.bash
    ```

3. run the ``cartesian_pose_example_controller`` under the ``franka_bringup`` package


    ```
    ros2 launch franka_bringup example.launch.py controller_name:=cartesian_pose_example_controller
    ```

4. single point test:
      ```
      ros2 topic pub /NS_1/cartesian_move std_msgs/msg/Float64MultiArray "{data: [0.6, 0, 0.2, 0.0, 1, 0.0, 0, 2]}" --once
      ```

     ``` 
     ros2 topic pub /NS_1/cartesian_move std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3, 0.5, 0.0, 0.9, 0.0, 0.1, 0.5]}" --once
     ```

    ```
    ros2 topic pub /NS_1/cartesian_move std_msgs/msg/Float64MultiArray "{data: [0.3, 0.4, 0.5, 0.0, 0.9, 0.0, 0.1, 0.3]}" --once
    ```

    ```
    ros2 topic pub /NS_1/cartesian_move std_msgs/msg/Float64MultiArray "{data: [0.3, 0.2, 0.6, 0.0, 0.9, 0.0, 0.1, 0.3]}" --once
    ```

## Terminal 2

1. create a new dir (whatever) and cd it 
    ```
    cd sigma_franka_ws/
    ```

2. put the ``sigma7`` dir under the ``src/`` of your mew dir

3. colcon build and source it
    ```
    colcon build 
    ```

    ```
    source install/setup.bash
    ```

4. run the sigma7 device
    ```
    ros2 run sigma7 sigma_main
    ```

## Terminal 3

1. put the ``omega_to_franka_cartesian.py`` under your new dir (not in ``src/``)

    ```
    cd sigma_franka_ws/
    ```

    ```
    source install/setup.bash
    ```

    ```
    python3 omega_to_franka_cartesian
    ```




## workspace limit of franka

![工作空间奇异构型1](https://github.com/user-attachments/assets/910e0893-3ab3-4c58-ad10-6aa1629615fa)

![工作空间奇异构型2](https://github.com/user-attachments/assets/37c4d354-a712-41bd-bb2d-63c73f06023c)

![正常工作空间](https://github.com/user-attachments/assets/494522b1-796c-475a-8429-1384b7712de3)


```
x: [0.3, 0.75]
y: [-0.6, 0.6]
z: [0, 0.6]
```

