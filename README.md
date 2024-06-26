# OA-Bug-Robot

![Platform](README.assets/Platform.jpg)

This repo contains our implementation of the OA-Bug algorithm on the Lingao robot. The platform includes at least the following (may work on other platforms):

- 64-bit Raspbian OS (Debian 10) on Raspberry Pi 4B with `wiringpi`

  - To install `wiringpi` on 64-bit Raspbian OS, you can use these commands:

    ```shell
    dpkg --add-architecture armhf
    sudo apt update
    sudo apt install libc6
    sudo apt --fix-broken install
    sudo apt install libc++6:armhf
    wget https://project-downloads.drogon.net/wiringpi-latest.deb
    sudo dpkg -i wiringpi-latest.deb
    gpio readall
    ```

- Python 3.6+

- ROS noetic

- `libqt5charts5-dev`

To run the robot, set up all the hardware and make sure `roscore` is running, then run `python3 robot_using_ros.py` to start.

For convenience, the robot can be monitored and controlled by a GUI, roughly following the MVC design pattern:

- `utils.py` contains some tools and classes to get data, which can be considered as Model in MVC;
- `views.py` contains components of the GUI;
- `robot.py` is the core logic of the OA-Bug algorithm, as the role of the Controller in MVC (for convenience, the function to update `ControlPanel` is implemented in the `Robot` class);
- `state.py` contains the abstract class for the state machine.

For the underlying control board, please see:

- [kevintsq/lingao_ros: ROS packages for the Lingao Robot. Forked from https://keaa.coding.net/public/lingaoros/lingao_ros/git/files. (github.com)](https://github.com/kevintsq/lingao_ros)
- [kevintsq/lingao_control_board: Source code and schematic diagrams of the control board with STM32F405RGT6 from Lingao. (github.com)](https://github.com/kevintsq/lingao_control_board)
