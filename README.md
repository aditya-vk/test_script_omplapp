## OMPLApp Test Scripts

# Dependency installation:

1. This package assumes you have ROS installed.
    * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
    * `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
    * `sudo apt-get update && sudo apt-get install ros-melodic-desktop-full`
    * Setup a catkin workspace.

2. Install DART for collision checking and physics simulations. The instructions are [here](https://dartsim.github.io/install_dart_on_ubuntu.html). This is how I do it:
	* `git clone` DART from GitHub. Checkout the v6.8.0 branch.
	* Assuming you have a catkin workspace setup, `catkin build dartsim` should build the package.

3. Install AIKIDO for motion planning interface. It also interfaces with DART.
	* `git clone http://github.com/personalrobotics/aikido`
	* To ensure you have all the required dependencies: `cd aikido && mkdir build && cd build && cmake ..`. Install the missing dependencies from apt.
	* AIKIDO has installation instructions on its readme but I personally generally avoid installing packages like dart etc. system-wide.
	* `catkin build aikido`

4. Finally build the current package:
	* `catkin build test_script_omplapp`

The executable is found under `catkin_ws/devel/lib/test_script_omplapp/<exectuable name>`.


To visualize the plan, you can open RViz and visualize the interactive marker under the topic dart.