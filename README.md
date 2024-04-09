# ROS2-Humble

# Requirements:
1. UBUNTU 22.04 LTS
2. ROS-Humble
4. Gazebo-garden
5. Ardupilot

# UBUNTU 22.04 LTS
Install and setup in Virtual box or dual boot.
https://ubuntu.com/download/desktop

# ROS-Humble
1. Referred from https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Steps to install and setup
  i. Set locale
   ```
    locale
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
   ```

   ii. Setup Sources
    ```
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

    iii. Install ROS
     ```
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop-full
     ```

     iv. Environment setup
     ```
     source /opt/ros/humble/setup.bash
     echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
     ```

3. Test by running ```ros2```
   Output will be:
   ```
   usage: ros2 [-h] [--use-python-default-buffering]
         Call `ros2 <command> -h` for more detailed usage. ...
   ros2 is an extensible command-line tool for ROS 2.
   options:
     -h, --help            show this help message and exit
     --use-python-default-buffering
                     Do not force line buffering in stdout and instead use
                     the python default buffering, which might be affected
                     by PYTHONUNBUFFERED/-u and depends on whatever stdout
                     is interactive or not
   Commands:
     action     Various action related sub-command
     bag        Various rosbag related sub-commands    
     component  Various component related sub-commands    
     daemon     Various daemon related sub-commands    
     doctor     Check ROS setup and other potential issues    
     interface  Show information about ROS interfaces    
     launch     Run a launch file    
     lifecycle  Various lifecycle related sub-commands    
     multicast  Various multicast related sub-commands    
     node       Various node related sub-commands    
     param      Various param related sub-commands    
     pkg        Various package related sub-commands    
     run        Run a package specific executable    
     security   Various security related sub-commands    
     service    Various service related sub-commands    
     topic      Various topic related sub-commands    
     wtf        Use `wtf` as alias to `doctor`        
     Call `ros2 <command> -h` for more detailed usage.
   ```

# Gazebo - garden
1. Referred from
  https://gazebosim.org/docs/garden/install_ubuntu
  https://gazebosim.org/docs/garden/ros_installation

2. Steps to install with ROS2:
   ```
   sudo apt-get update
   sudo apt-get install lsb-release wget gnupg
   ```
   ```
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
   ```
   ```
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   ```
   ```
   sudo apt-get update
   sudo apt-get install gz-garden
   ```
   Install ros_gz from the non official binary packages from apt:
   ```
   sudo apt-get install ros-humble-ros-gzgarden
   ```

3. Check installation:
   ```
   gz sim -v4 -r shapes.sdf
   ```

# Ardupilot Plugin
1. Referred from https://ardupilot.org/dev/docs/sitl-with-gazebo.html#sitl-with-gazebo

2. Steps to install:
  i. To install ardupilot_gazebo plugin
      Install additional dependencies
   ```
   sudo apt update
   sudo apt install libgz-sim7-dev rapidjson-dev
   ```
   ```
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
   ```
   ```
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   ```
   ```
   sudo apt-get update
   sudo apt-get install gz-garden
   ```
   Install ros_gz from the non official binary packages from apt:
   ```
   sudo apt-get install ros-humble-ros-gzgarden
   ```
3. Check installation:
    ```
    gz sim -v4 -r shapes.sdf
    ```

# Ardupilot Plugin
1. Referred from https://ardupilot.org/dev/docs/sitl-with-gazebo.html#sitl-with-gazebo

2. Steps to install:

  i. To install ardupilot_gazebo plugin
    Install additional dependencies
    ```
    sudo apt update
    sudo apt install libgz-sim7-dev rapidjson-dev
    ```

  ii. Create a workspace folder and clone the repository:
  ```
  mkdir -p gz_ws/src && cd gz_ws/src
  git clone https://github.com/ArduPilot/ardupilot_gazebo
  ```

  iii. Build the plugin:
  ```
  export GZ_VERSION=garden
  cd ardupilot_gazebo
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j4
  ```

  iv. Configure the environment:
  ```
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
  export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
  ```
  ```
  echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc
  echo "export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc
  ```

  v. In a new terminal install ardupilot (https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
    Open terminal in Home location
    ```
    git clone https://github.com/ArduPilot/ardupilot -b master
    cd ardupilot
    ```
    
  Install some required packages:
    ```
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    ```
  
  Reload the path (log-out and log-in to make permanent):
    ```
    . ~/.profile
    ```
    
  waf calls (do not use sudo):
    ```
    ./waf configure --board CubeBlack
    ./waf copter
    ```
  
  clean:
    ```
    ./waf clean
    ```

# Running gazebo SITL
1. Terminal - 1 : To run gazebo world:
   ```
   gz sim -v4 -r iris_runway.sdf
   ```

2. Terminal - 2 : To connect to ardupilot plugin:
   ```
   cd ardupilot
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
   ```
