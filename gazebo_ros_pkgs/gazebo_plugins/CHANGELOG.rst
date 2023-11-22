^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.7.0 (2022-06-13)
------------------
* Add friction coefficient parameters (`#1393 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1393>`_)
* GPS sensor plugin publishing velocity (`#1371 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1371>`_) (`#1387 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1387>`_)
* Contributors: Jacob Perron, Jenn Nguyen, Marcel Dudek

3.6.0 (2022-05-10)
------------------
* Initialize wheel slip parameters directly from SDF (`#1365 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1365>`_)
* Fix test failures (`#1380 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1380>`_)
* gazebo_ros_ft_sensor_demo.world: use world solver (`#1354 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1354>`_)
* gazebo_ros_wheel_slip: set lateral slip to zero at low speed (`#1338 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1338>`_)
* gazebo_ros_wheel_slip: publish wheel slip (`#1331 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1331>`_)
* Add slip values for individual wheels (`#1312 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1312>`_)
* Default slip values fix for wheel slip plugin (`#1308 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1308>`_)
* Add runtime warning when user sets use_sim_time parameter
* Fix warnings when building against the latest sources (`#1282 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1282>`_)
* Add method to get ROS node from GazeboRosCameraPlugin (`#1299 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1299>`_)
* Improve robustness of joint state publisher test (`#1259 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1259>`_)
* Avoid rejecting QoS overrides parameters (`#1258 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1258>`_)
* Publish with QoS reliable as default (`#1224 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1224>`_) (`#1235 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1235>`_)
* Contributors: Aditya Pande, Audrow Nash, Brett Downing, Chris Lalancette, Daisuke Nishimatsu, Dharini Dutia, Jacob Perron, Steve Peters

3.5.2 (2021-03-15)
------------------
* gazebo_ros_camera: Added accessor methods for camera properties (`#1246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1246>`_)
  * Added accessor methods to gazebo_ros_camera for subclass access to camera properties.
  * Removed ros node accessor; no longer needed
  * Removed const return types and added vector include to header
  Co-authored-by: kbjeppes <kaden.b.jeppesen@nasa.gov>
* Fix tests for cyclonedds (`#1228
  <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1228>`_) The
  default RMW implementation changed recently and some tests are now failing.
  This fixes the tests.  * Use KeepLast(1) with transient_local in tests There
  are some QoS incompatibilities in some tests that use SystemDefaultsQoS, so
  this changes them to use KeepLast(1) with transient_local instead. This fixes
  some of the test failures but not all.  * test_sim_time: allow more startup
  messages * Fix QoS and initialization of joint state pub test
* Make p3d offset element names singular (`#1210 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1210>`_)
  * Make p3d offset element names singular
  - <xyz_offsets> is renamed to <xyz_offset>
  - <rpy_offsets> is renamed to <rpy_offsets>
  The old names can still be used, but are deprecated.
  This is more consistent with the naming convention used in ROS 1 versions.
  * Add test for deprecated functionality
* Contributors: Jacob Perron, Steve Peters, kjeppesen1

3.5.1 (2020-11-25)
------------------
* colcon.pkg: build gazebo first in colcon workspace (`#1192 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1192>`_)
  Add a colcon.pkg file to gazebo_dev with gazebo's cmake project
  name "Gazebo" listed as a dependency to support building
  gazebo from source in a colcon workspace.
  * Add colcon.pkg files for other packages
  Copy colcon.pkg to gazebo_ros, gazebo_plugins, and
  gazebo_ros_control so that --merge-install won't be required.
  Signed-off-by: Steve Peters <scpeters@openrobotics.org>
* Fixed Parameterized testing on Rolling (`#1184 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1184>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* adding buffer to gzebo ros hand of god plugin (`#1179 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1179>`_)
  Fixes `#1178 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1178>`_.
* [gazebo_plugins] address warnings (`#1151 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1151>`_)
  * use .c_str() for variadic template
  * silence warnings
  Signed-off-by: Karsten Knese <Karsten1987@users.noreply.github.com>
  Signed-off-by: Steve Peters <scpeters@openrobotics.org>
* Port wheel slip plugin to ros2 (forward port from eloquent) (`#1148 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1148>`_)
  Forward port of wheel slip plugin from eloquent (`#1099 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1099>`_)
  to foxy.
  It includes a similar change to `#1111 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1111>`_, which moved the
  WheelSlipPlugin::Load call before the parameter callback
  so that the values from SDF/URDF are set first.
  Then the callback is changed to ignore negative values, and the
  default slip parameter values are set to -1, so that the SDF/URDF
  values are still preferred unless a different parameter value
  is specified in a launch file.
  Signed-off-by: Steve Peters <scpeters@openrobotics.org>
* Added ignition common profiler to ros2 (`#1141 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1141>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, Karsten Knese, Steve Macenski, Steve Peters

3.5.0 (2020-06-19)
------------------
* Merge pull request `#1130 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1130>`_ from ros-simulation/foxy_tests
  Fix all Foxy tests
* Merge pull request `#1129 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1129>`_ from ros-simulation/e_to_f_june_2020
  Eloquent ➡️ Foxy
* Apply acceleration until both left and right reach targetspeed (`#1009 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1009>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* use target include directories (`#1040 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1040>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Dashing -> Eloquent
* [forward port] Image publishers use SensorDataQoSProfile (`#1031 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1031>`_) (`#1052 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1052>`_)
  All other sensor publishers were updated previously to use the same profile (`#926 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/926>`_).
  I'm not sure if the image publishers were overlooked or the image_transport API didn't
  support setting the QoS profile at the time.
* Fix cppcheck errors (`#1123 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1123>`_)
  cppcheck 1.90 complains about syntax errors even though it is valid C++ code.
  This refactoring fixes the reported errors.
* Replace deprecated parameters callback API (`#1121 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1121>`_)
  rclcpp::Node now supports multiple parameter callbacks, so we do not need to worry about overwriting an existing callback.
  This change fixes compile time deprecation warnings since ROS Foxy.
* Replace deprecated image_common headers (`#1122 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1122>`_)
  This fixes compile-time deprecation warnings.
* Measure IMU orientation with respect to world (ros2) (`#1064 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1064>`_)
  * Measure IMU orientation with respect to world (`#1058 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1058>`_)
  Report the IMU orientation from the sensor plugin with respect to the world frame.
  This complies with convention documented in REP 145: https://www.ros.org/reps/rep-0145.html
  In order to not break existing behavior, users should opt-in by adding a new SDF tag.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
  * IMU sensor: comply with REP 145 by default
  Change default value of initial_orientation_as_reference to false
  and print deprecation warning if user explicitly sets it to true.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* Make QoS for publishers and subscriptions configurable  (`#1092 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1092>`_)
  * Make QoS for publishers and subscriptions configurable
  Whenever a plugin creates a ROS publisher or subscription, use the QoS profile provided by the node for the given topic.
  This enables users to override the QoS settings in SDF.
  Depends on https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1091.
* [eloquent] Fix Windows build. (`#1077 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1077>`_)
  * Adding Windows bringup.
* Gazebo 11 for Foxy (`#1093 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1093>`_)
  * Gazebo 11 for Foxy
* 3.3.5
* Backport Gazebo11/Bionic fix for boost variant (`#1102 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1102>`_)
* Measure IMU orientation with respect to world (dashing) (`#1065 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1065>`_)
  Report the IMU orientation from the sensor plugin
  with respect to the world frame.
  This complies with convention documented in REP 145:
  https://www.ros.org/reps/rep-0145.html
  In order to not break existing behavior,
  users should opt-in by adding a new SDF tag.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* Uncrustify (`#1060 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1060>`_)
  Style changes to conform to the new default setting introduced in https://github.com/ament/ament_lint/pull/210.
  Arguments that do not fit on one line must start on a new line.
* Contributors: Jacob Perron, Jose Luis Rivero, Karsten Knese, Louise Poubel, Sean Yen, Steve Peters, Steven Peters, scgroot

3.4.4 (2020-05-08)
------------------
* Backport Gazebo11/Bionic fix for boost variant (`#1103 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1103>`_)
* Measure IMU orientation with respect to world (`#1058 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1058>`_)
  Report the IMU orientation from the sensor plugin with respect to the world frame.
  This complies with convention documented in REP 145: https://www.ros.org/reps/rep-0145.html
  In order to not break existing behavior, users should opt-in by adding a new SDF tag.
* Contributors: Jose Luis Rivero, Steven Peters, Jacob Perron

3.4.3 (2020-02-18)
------------------
* Image publishers use SensorDataQoSProfile (`#1031 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1031>`_)
  All other sensor publishers were updated previously to use the same profile (`#926 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/926>`_).
  I'm not sure if the image publishers were overlooked or the image_transport API didn't
  support setting the QoS profile at the time.
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [backport] Backport multicamera to dashing (`#984 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/984>`_)
  * [backport] Backport multicamera to dashing
  * fix test - use correct world
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding option to select the frame where the force will be applied (`#978 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/978>`_)
  * Adding option to select the frame where the force will be applied
  A new parameter was added on the plugin with the options 'world' and 'link' frame.
  The default value is 'world'.
  Internally the AddRelativeForce() and torque functions are used instead of the AddForce() when the body option is selected.
  * Modifying force test for the 'world' frame, and adding test for the force on the 'link' frame
  The new world file starts with the box rotated.
  * Fix cpplint and uncrustify on force plugin files
  * Removing OnUpdateRelative() from the force plugin
  This function could potentially break the ABI, therefore is been removed.
  * body -> link, warn -> info, more examples
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding GPS plugin (`#982 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/982>`_)
  * Adding gps plugin sensor
  * Adding test for the gps plugin
  * Adding GPS world demo and other small text corrections
* [ros2] Backport depth camera to dashing (`#967 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/967>`_)
  * [ros2] Backport depth camera to dashing
  * don't install header that will be removed
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * fix linting error
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port vacuum gripper to ROS2 (`#960 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/960>`_)
  * [ros2] Port vacuum gripper to ROS2
  * Fix gripper forces
  * Add option to set max_distance
  Change SetForce -> Add Force
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
  Minor fixes
* Merge pull request `#977 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/977>`_ from ros-simulation/backport
  [backport] ros2 -> dashing
* fix video test
  Signed-off-by: chapulina <louise@openrobotics.org>
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
  * [ros2] Port bumper sensor to ROS2
  * Add author name
  * Minor fixes and add contact msg conversion
  * Remove unused header includes
* Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_) (`#937 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/937>`_)
  * Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_)
  * Fix crash on shutdown
  * Fix gazebo node destructor
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  backport remove noe fix and re-enable distortion tests
  * improve robustness of joint state pub test
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Add Gazebo builtin plugins to LD_LIBRARY_PATH (`#974 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/974>`_)
  * Add Gazebo builtin plugins to LD_LIBRARY_PATH
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * cross-platform
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
  * [ros2] Port hand of god to ROS2
  * Minor fixes
* [ros2] Port harness to ROS2 (`#944 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/944>`_)
* 3.3.2
* changelog
* [ros2] Add ackermann drive plugin (`#947 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/947>`_)
  * [ros2] Add ackermann drive plugin
  * Minor fixes
  Use gazebo database model
  * Update example usage
  * Fix TF for demo
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
  * [ros2] Port planar move to ROS2
  * Add test for pose conversion
* [ros2] Port projector to ROS2 (`#956 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/956>`_)
  * [ros2] Port projector to ROS2
  * fix small typo
* Merge pull request `#945 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/945>`_ from shiveshkhaitan/elevator
  [ros2] Port elevator to ROS2
* [ros2] Fix test for diff drive (`#951 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/951>`_)
* [ros2] Port elevator to ROS2
* Contributors: Jacob Perron, Jonathan Noyola, Louise Poubel, Shivesh Khaitan, alexfneves, chapulina

3.4.2 (2019-11-12)
------------------
* Merge branch 'ros2' into eloquent
* [ros2] Add remapping tag (`#1011 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1011>`_)
  * add --ros-args and a remapping element for ros arguments
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * keep backward compatibility
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * update docs and world file accordingly
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * remap all the things :fist_raised:
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* generate a .dsv file for the environment hook
* Contributors: Dirk Thomas, Louise Poubel, Mikael Arguedas

3.4.1 (2019-10-10)
------------------
* generate a .dsv file for the environment hook
* Contributors: Dirk Thomas

3.4.0 (2019-10-03)
------------------
* Add Gazebo builtin plugins to LD_LIBRARY_PATH (`#974 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/974>`_)
  * Add Gazebo builtin plugins to LD_LIBRARY_PATH
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding option to select the frame where the force will be applied (`#978 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/978>`_)
  * Modifying force test for the 'world' frame, and adding test for the force on the 'link' frame
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding GPS plugin (`#982 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/982>`_)
* fix multi_camera_plugin on windows (`#998 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/998>`_)
* Merge pull request `#980 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/980>`_ from shiveshkhaitan/forward_port
  [forward_port] dashing -> ros2
* [ros2] Port vacuum gripper to ROS2 (`#960 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/960>`_)
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
* fix video test
  Signed-off-by: chapulina <louise@openrobotics.org>
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
  * [ros2] Port hand of god to ROS2
* [ros2] Port harness to ROS2 (`#944 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/944>`_)
* [ros2] Add ackermann drive plugin (`#947 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/947>`_)
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
* [ros2] Port projector to ROS2 (`#956 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/956>`_)
* [ros2] Fix test for diff drive (`#951 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/951>`_)
* [ros2] Port elevator to ROS2
* [ros2] Dynamic reconfigure for gazebo_ros_camera (`#940 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/940>`_)
* [ros2] Port multicamera to ros2 (`#939 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/939>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
* [ros2] Port depth camera to ROS2 (`#932 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/932>`_)
* Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_) (`#937 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/937>`_)
* [ros2] Port skid_steer_drive to ROS2 (`#927 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/927>`_)
* [ros2] Port F3d and FTSensor plugin to ros2 (`#921 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/921>`_)
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Jonathan Noyola, Shivesh Khaitan, alexfneves, chapulina

3.3.5 (2020-05-08)
------------------
* Backport Gazebo11/Bionic fix for boost variant (`#1102 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1102>`_)
* Measure IMU orientation with respect to world (dashing) (`#1065 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1065>`_)
  Report the IMU orientation from the sensor plugin with respect to the world frame.
  This complies with convention documented in REP 145:
  https://www.ros.org/reps/rep-0145.html
  In order to not break existing behavior,users should opt-in by adding a new SDF tag.
* Contributors: Jose Luis Rivero, Steven Peters, Jacob Perron

3.3.4 (2019-09-18)
------------------
* fix multi_camera_plugin on windows (`#999 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/999>`_)
* Contributors: Jonathan Noyola

3.3.3 (2019-08-23)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [backport] Backport multicamera to dashing (`#984 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/984>`_)
  * [backport] Backport multicamera to dashing
  * fix test - use correct world
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding option to select the frame where the force will be applied (`#978 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/978>`_)
  * Adding option to select the frame where the force will be applied
  A new parameter was added on the plugin with the options 'world' and 'link' frame.
  The default value is 'world'.
  Internally the AddRelativeForce() and torque functions are used instead of the AddForce() when the body option is selected.
  * Modifying force test for the 'world' frame, and adding test for the force on the 'link' frame
  The new world file starts with the box rotated.
  * Fix cpplint and uncrustify on force plugin files
  * Removing OnUpdateRelative() from the force plugin
  This function could potentially break the ABI, therefore is been removed.
  * body -> link, warn -> info, more examples
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Adding GPS plugin (`#982 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/982>`_)
  * Adding gps plugin sensor
  * Adding test for the gps plugin
  * Adding GPS world demo and other small text corrections
* [ros2] Backport depth camera to dashing (`#967 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/967>`_)
  * [ros2] Backport depth camera to dashing
  * don't install header that will be removed
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * fix linting error
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port vacuum gripper to ROS2 (`#960 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/960>`_)
  * [ros2] Port vacuum gripper to ROS2
  * Fix gripper forces
  * Add option to set max_distance
  Change SetForce -> Add Force
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
  Minor fixes
* Merge pull request `#977 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/977>`_ from ros-simulation/backport
  [backport] ros2 -> dashing
* fix video test
  Signed-off-by: chapulina <louise@openrobotics.org>
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
  * [ros2] Port bumper sensor to ROS2
  * Add author name
  * Minor fixes and add contact msg conversion
  * Remove unused header includes
* Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_) (`#937 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/937>`_)
  * Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_)
  * Fix crash on shutdown
  * Fix gazebo node destructor
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  backport remove noe fix and re-enable distortion tests
  * improve robustness of joint state pub test
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Add Gazebo builtin plugins to LD_LIBRARY_PATH (`#974 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/974>`_)
  * Add Gazebo builtin plugins to LD_LIBRARY_PATH
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * cross-platform
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
  * [ros2] Port hand of god to ROS2
  * Minor fixes
* [ros2] Port harness to ROS2 (`#944 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/944>`_)
* Contributors: Shivesh Khaitan, alexfneves, chapulina

3.3.2 (2019-07-31)
------------------
* [ros2] Add ackermann drive plugin (`#947 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/947>`_)
  * [ros2] Add ackermann drive plugin
  * Minor fixes
  Use gazebo database model
  * Update example usage
  * Fix TF for demo
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
  * [ros2] Port planar move to ROS2
  * Add test for pose conversion
* [ros2] Port projector to ROS2 (`#956 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/956>`_)
  * [ros2] Port projector to ROS2
  * fix small typo
* Merge pull request `#945 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/945>`_ from shiveshkhaitan/elevator
  [ros2] Port elevator to ROS2
* [ros2] Fix test for diff drive (`#951 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/951>`_)
* [ros2] Port elevator to ROS2
* [ros2] Port skid_steer_drive to ROS2 (`#927 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/927>`_)
  * [ros2] Port skid_steer_drive to ROS2
  Integrate skid steer drive into diff drive
  * Reverted to original diff drive
  * Delete skid steer from .ros1_unported
  * Fix for diff drive changed api
  * Add support to specify odom covariance
* [ros2] Port F3d and FTSensor plugin to ros2 (`#921 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/921>`_)
  * [ros2] Port F3d plugin to ros2
  * Merge ft_sensor and f3d_plugin
  * Delete ft_sensor from .ros1_unported
  * Minor fixes
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Shivesh Khaitan, chapulina

3.3.1 (2019-05-30)
------------------
* qos dashing api for video plugin (`#929 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/929>`_)
  * qos dashing api for video plugin
  * disable video test unless display is enabled
* [ros2] Port tricycle_drive plugin to ros2 (`#917 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/917>`_)
  * [ros2] Port tricycle_drive plugin to ros2
  * Set feasible test cmd_vel
  * qos dashing api for tricycle
  * Minor fixes
  * Fix tricycle behaviour on gazebo reset
* Contributors: Shivesh Khaitan

3.3.0 (2019-05-21)
------------------
* use latest dashing api (`#926 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/926>`_)
  * [gazebo_ros] use qos
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * [gazebo_ros] avoid unused warning
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * [gazebo_plugins] use qos
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * allow_undeclared_parameters
  * fix tests
  * forward port pull request `#901 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/901>`_
* [ros2] Port video plugin to ros2 (`#899 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/899>`_)
  * [ros2] Port video plugin to ros2
  * Fix test for gazebo_ros_video
* use `.c_str()` for variadic template (`#914 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/914>`_)
  Not sure why this never was a problem, but I had to fix this in order to make it compile on OSX.
* [ros2] Fix diff_drive error message (`#882 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/882>`_)
* Fix Windows conflicting macros and missing usleep (`#885 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/885>`_)
  * Fix conflicting Windows macros and missing usleep
  * fix spacing
  * fix spacing again
  * remove lint
* gazebo_plugins: Port the gazebo_ros_p3d plugin (`#845 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/845>`_)
  * Port the gazebo_ros_p3d plugin
  * Address most of the review feedback. A couple items remain
  * Remove the model\_ member variable since it was just and alias for _parent
  * Use OnUpdate instead to get the UpdateInfo through the callback parameter
  * demo, test, and a bit more cleaning up
  * linters
* [ros2] ENABLE_DISPLAY_TESTS, and make camera tests more robust (`#854 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/854>`_)
* Contributors: Jonathan Noyola, Karsten Knese, Michael Jeronimo, Romain Reignier, Shivesh Khaitan, chapulina

3.1.0 (2018-12-10)
------------------
* [ros2] Camera and triggered camera (`#827 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/827>`_)
  * move gazebo_ros_camera and some functionality from gazebo_ros_camera_utils, needs master branch of image_transport and message_filters, not functional, but compiling
  * port PutCameraData, needs common_interfaces PR `#58 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/58>`_
  * move camera worlds, fix compilation, image can be seen on RViz
  * Port camera test: simplify world, use ServerFixture for better control and not to need launch - test is hanging on exit, not sure why
  * fix test hanging on exit
  * port camera16bit test and fix world copying on make
  * Start porting camera distortion tests: must port cam_info, 2nd test failing
  * sortout camera_name and frame_name
  * Port gazebo_ros_camera_triggered as part of gazebo_ros_camera, with test
  * Use camera_info_manager from branch ci_manager_port_louise, enable barrel distortion test - passes but segfaults at teardown, could be a problem with having 2 plugins side-by-side.
  * linters and comment out crashing test
  * Demo worlds, doxygen, more node tests
  * Use image_transport remapping
  * adapt to new image_transport pointer API
  * new API
* Contributors: chapulina

3.0.0 (2018-12-07)
------------------
* Reliable QoS with depth of 1 (`#819 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/819>`_)
* Switch to use sensor_data qos setting for short queue sizes. (`#815 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/815>`_)
  * Switch to use sensor_data qos setting for short queue sizes.
  * Use same QoS profile on test
* [ros2] Port diff_drive plugin to ros2 (`#806 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/806>`_)
  * copy gazebo_ros_diff_drive files from unported
  * Fix copy and paste error for exporting  gazebo_ros_joint_state_publisher
  * Add gazebo_ros_diff_drive to CMakeLists.txt
  * Basic structures updated
  includes updated
  include guards updated
  CMake rules added
  Not compiling yet
  * starting deboostifying
  updating lock
  header passing compile
  diff drive plugin compiling
  clear all references to callback queue
  * pimpl, remove joint state publisher
  * documentation, add TF publishers - commands and publishers work, but visualization on RViz is jerky, must check
  * pass linters
  * check that reset works now, rename params, add missing package
  * remap topics, add pub/sub test
  * sleep longer to see if it passes on Jenkins
* Remove node_name from <ros> SDF tag (`#804 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/804>`_)
  * Rename Node::Create to Node::Get
  * Node::Get without node name
  * Remove node_name support from SDF
  * wip get name from plugin name
  * Remove node name argument (will be inferred from sdf)
  * fix tests and implement static shared node
  * Adding test file
* [ros2] Remove unnecessary IMU include (`#805 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/805>`_)
  * removing redundant dependencies
  * Clear unnecessary include in imu_sensor header
* [ros2] Split conversions into headers specific to message packages (`#803 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/803>`_)
  * Tests depend on sensor_msgs
  * Move conversions to different headers to minimise deps brought in
  * Remove conversions namespace
  * Include updates
  * Update message package dependencies
  gazebo_ros doesn't need sensor_msgs or geometry_msgs anymore
  * Export msg pacakges so downstream packages depend
  * Include msg headers used directly
  * removing redundant dependencies
  * fix build and cpplint
* working demo, notes and warnings about issues
* Add more examples, need to debug some
* tweaks to includes
* Test correctness of ray_sensor intensity
* Add ray_sensor demo
* Verify correctness of gazebo_ros_ray_sensor output
* Simplify ray_sensor using gazebo_ros conversions
* Add gazebo_ros_ray_sensor
* [ros2] Add noise to imu test (`#801 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/801>`_)
  * Add noise to IMU test world
  * Remove bias
  * Relax test tolerance
* [ros2] Port gazebo_ros_imu_sensor (`#793 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/793>`_)
  * Move files to prepare for imu_sensor ROS2 port
  * Port gazebo_ros_imu_sensor
  * Address IMU Sensor PR comments
  * Remove empty <imu> tag
  * document that always_on is required
  * alphabetical order includes
  * Step far forward instead of multiple small steps
  * Fix test_conversions not finding quaternion.hpp
  * Apply force longer; check IMU values; robust to negative linear accel
  * linter fixup
* [ros2] gazebo_ros_joint_state_publisher (`#795 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/795>`_)
  * Port joint_state_publisher, copyright failing checker, still need to add a test
  * Fix copyright
  * Tests for joint state publisher
  * cleanup
  * depend on sensor_msgs
  * Use node's logger
* PR feedback
* Add test using ServerFixture
* conversions
* Convert plugin and add test world
* move gazebo_ros_force files
* remove target_link_libraries
* improve example, add demo world, fix sdf warnings
* Port gazebo_ros_template and add more instructions
* Boostrap gazebo_plugins as ament package
* Move ros1 gazebo_plugins files into root
* Contributors: Kevin Allen, Louise Poubel, Tully Foote, chapulina, dhood

2.8.4 (2018-07-06)
------------------
* Fix various xacro/xml issues with tests
* Fix handling of boolean values since Gazebo API returns
  'true'/'false' as '1'/'0' strings
* Add auto_distortion parameter to camera utils
* Corrected depth camera plugin initialization (#748)
  * Initialize depth_image_connect_count\_ to 0
  * Removed duplicate line in CMakeLists.txt
* Fix melodic compiler warnings (#744)
  * Fix model_state_test. -v means --version not --verbose
  * fix gazebo9 warnings by removing Set.*Accel calls
  * gazebo_plugins: don't use -r in tests
* add missing distortion test worlds
* fix 16bit test name
* test for triggered_camera
* update copyright dates and remove copied comments
* remove compiler directives for old gazebo versions
* use correct timestamp for images
* adds triggered cameras and multicameras
* Contributors: Jose Luis Rivero, Kevin Allen, Martin Ganeff, Morgan Quigley, Steven Peters, Timo Korthals, iche033

2.8.3 (2018-06-04)
------------------
* End of legacy for diff drive plugin (`#707 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/707>`_)
  This PR ends with the option to set legacy in a ROS parameter.
  In old versions of the code the right and left wheel were changed
  to fix a former code issue. To fix an old package you have to
  exchange left wheel by the right wheel.
* Remove gazebo_ros_joint_trajectory plugin (`#708 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/708>`_)
* Add publishOdomTF flag (`#692 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/692>`_) (`#727 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/727>`_)
* DIFF DRIVE: wheel odometry twist is child frame (`#719 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/719>`_)
* ROS UTILS: initialize rosnode\_ in alternative constructor to avoid segfault `#478 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/478>`_ (`#718 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/718>`_)
* Contributors: Jose Luis Rivero, Kevin Allen

2.8.2 (2018-05-09)
------------------
* Fix the build on Ubuntu Artful. (`#715 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/715>`_)
  Artful has some bugs in its cmake files for Simbody that
  cause it to fail the build.  If we are on artful, remove
  the problematic entries.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

2.8.1 (2018-05-05)
------------------
* Update version to 2.8.0
* Fix sensors after time reset (lunar-devel) (`#705 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/705>`_)
  * camera plugin keeps publishing after negative sensor update interval
  World resets result in a negative time differences between current world
  time and the last recorded sensor update time, preventing the plugin
  from publishing new frames. This commit detects such events and resets
  the internal sensor update timestamp.
  * block_laser, range, and joint_state_publisher keep publishing after clock reset
  * p3d keeps publishing after clock reset
* Support 16-bit cameras (lunar-devel) (`#700 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/700>`_)
  * extend camera util to support 16 bit rgb image encoding
  * support 16 bit mono
  * add test for 16-bit camera
  * update skip\_
  * move camera test to camera.h, add camera16bit.cpp
* Fix `#612 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612>`_ for Gazebo9 (lunar-devel) (`#699 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/699>`_)
  * Fix `#612 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612>`_ for Gazebo9
  This commit fixes `#612 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612>`_, but only for Gazebo9. Fixing it for Gazebo7 (the version used in ROS Kinetic) requires the following PR to be backported to Gazebo 7 and 8:
* gazebo_plugins: unique names for distortion tests (lunar-devel) (`#686 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/686>`_)
  * gazebo_plugins: unique names for distortion tests
  * Missing test files
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------
* Adding velocity to joint state publisher gazebo plugin (`#671 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/671>`_)
* Fix last gazebo8 warnings! (lunar-devel) (`#664 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/664>`_)
* Fix gazebo8 warnings part 7: retry `#642 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/642>`_ on lunar (`#660 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/660>`_)
* gazebo8 warnings: ifdefs for Get.*Vel() (`#655 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/655>`_)
* Fix gazebo8 warnings part 8: ifdef's for GetWorldPose (lunar-devel) (`#652 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/652>`_)
* for gazebo8+, call functions without Get (`#640 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/640>`_)
* Fix conflict (`#647 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/647>`_)
* Contributors: Jose Luis Rivero, Steven Peters

2.7.3 (2017-12-11)
------------------
* Fix gazebo8 warnings part 4: convert remaining local variables in plugins to ign-math (lunar-devel) (`#634 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/634>`_)
* Fix gazebo8 warnings part 3: more ign-math in plugins (lunar-devel) (`#632 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/632>`_)
* Fix gazebo8 warnings part 2: replace private member gazebo::math types with ignition (lunar-devel) (`#630 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/630>`_)
* Replace Events::Disconnect* with pointer reset (`#626 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/626>`_)
* joint_state_publisher: error in case a joint is not found (`#609 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/609>`_)
* Contributors: Jose Luis Rivero, Kenneth Blomqvist

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Fixes for compilation and warnings in Lunar-devel  (`#573 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/573>`_)
  Multiple fixes for compilation and warnings coming from Gazebo8 and ignition-math3
* Add an IMU sensor plugin that inherits from SensorPlugin (`#363 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/363>`_)
* Less exciting console output (`#561 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/561>`_)
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Alessandro Settimi, Dave Coleman, Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------
* Revert catkin warning fix (`#567 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/567>`_)
  Many regressions in third party software (see https://github.com/yujinrobot/kobuki_desktop/issues/50)
* Contributors: Jose Luis Rivero

2.5.11 (2017-04-18)
-------------------
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning.
  Note that after the change problems could appear related to -lpthreads
  errors. This is an known issue related to catkin:
  https://github.com/ros/catkin/issues/856

* Fix: add gazebo_ros_range to catkin package libraries (`#558 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/558>`_)
* Contributors: Christoph Rist, Dave Coleman

2.5.10 (2017-03-03)
-------------------
* Revert catkin warnings to fix regressions (problems with catkin -lpthreads errors)
  For reference and reasons, please check:
  https://discourse.ros.org/t/need-to-sync-new-release-of-rqt-topic-indigo-jade-kinetic/1410/4
  * Revert "Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)"
  This reverts commit 5a0305fcb97864b66bc2e587fc0564435b4f2034.
  * Revert "Fix gazebo and sdformat catkin warnings"
  This reverts commit 11f95d25dcd32faccd2401d45c722f7794c7542c.
* Fix destructor of GazeboRosVideo (`#547 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/547>`_)
* Less exciting console output (`#549 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/549>`_)
* Fix SDF namespacing for Video Plugin (`#546 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/546>`_)
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)
* Fix timestamp issues for rendering sensors (kinetic-devel)
* Namespace console output (`#543 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/543>`_)
* Adding depth camera world to use in test to make depth camera have right timestamp `#408 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/408>`_- appears to be working (though only looking at horizon) but getting these sdf errors:
* `#408 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/408>`_ Make the multi camera timestamps current rather than outdated, also reuse the same update code
* Fix merge with kinetic branch
* `#408 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/408>`_ Making a test for multicamra that shows the timestamps are currently outdated, will fix them similar to how the regular camera was fixed.
* Fix for issue `#408 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/408>`_. The last measurement time is the time that gazebo generated the sensor data, so ought to be used. updateRate doesn't seem that useful.
  The other cameras need similar fixes to have the proper timestamps.
* Bugfix: duplicated tf prefix resolution
* fill in child_frame_id of odom topic
* Fix gazebo and sdformat catkin warnings
* Contributors: Dave Coleman, Jose Luis Rivero, Kei Okada, Lucas Walter, Yuki Furuta

2.5.8 (2016-12-06)
------------------
* Fix camera distortion coefficients order. Now {k1, k2, p1, p2, k3}
* Added an interface to gazebo's harness plugin
* Contributors: Enrique Fernandez, Steven Peters, Nate Koenig

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------
* fix gazebo7 deprecation warnings on kinetic
* Contributors: Steven Peters

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Accept /world for the frameName parameter in gazebo_ros_p3d
* Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* Update maintainer for Kinetic release
* use HasElement in if condition
* Contributors: Hugo Boyer, Jackie Kay, Jose Luis Rivero, Steven Peters, William Woodall, Yuki Furuta

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------
* Fix row_step of openni_kinect plugin
* remove duplicated code during merge
* merging from indigo-devel
* Merge pull request `#368 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/368>`_ from l0g1x/jade-devel
  Covariance for published twist in skid steer plugin
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
  cherry-picked from indigo-devel
  Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Publish organized point cloud from openni_kinect plugin
* Added covariance matrix for published twist message in the skid steer plugin, as packages such as robot_localization require an associated non-zero covariance matrix
* Added a missing initialization inside Differential Drive
* 2.4.9
* Generate changelog
* Merge pull request `#335 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/335>`_ from pal-robotics-forks/add_range_sensor_plugin
  Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Import changes from jade-branch
* Add range world and launch file
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, John Hsu, Jose Luis Rivero, Kentaro Wada, Krystian, Mirko Ferrati, Steven Peters, hsu

2.5.1 (2015-08-16)
------------------
* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added gazebo version check
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev (`#323 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323>`_)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added gazebo version check
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* run_depend on libgazebo5-dev instead of gazebo5
* Changed the rosdep key for gazebo to gazebo5, for Jade Gazebo5 will be used.
* Contributors: Steven Peters, William Woodall

2.4.9 (2015-08-16)
------------------
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Import changes from jade-branch
* Add range world and launch file
* Add ifdefs to fix build with gazebo2
* Use Joint::SetParam for joint velocity motors
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, Jose Luis Rivero, Steven Peters

2.4.8 (2015-03-17)
------------------
* fixed mistake at calculation of joint velocity
* [gazebo_ros_diff_drive] force call SetMaxForce since this Joint::Reset in gazebo/physics/Joint.cc reset MaxForce to zero and ModelPlugin::Reset is called after Joint::Reset
* add PointCloudCutoffMax
* Contributors: Kei Okada, Michael Ferguson, Sabrina Heerklotz

2.4.7 (2014-12-15)
------------------
* fix missing ogre flags: removed from gazebo default (5.x.x candidate) cmake config
* Fixing handling of non-world frame velocities in setModelState.
* fix missing ogre flags (removed from gazebo cmake config)
* change header to use opencv2/opencv.hpp issue `#274 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/274>`_
* Update Gazebo/ROS tutorial URL
* Merge pull request `#237 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/237>`_ from ros-simulation/update_header_license
  Update header license for Indigo
* Contributors: John Hsu, Jose Luis Rivero, Robert Codd-Downey, Tom Moore, hsu

2.4.6 (2014-09-01)
------------------
* Update gazebo_ros_openni_kinect.cpp
* merging from hydro-devel into indigo-devel
* Merge pull request `#204 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/204>`_ from fsuarez6/hydro-devel
  gazebo_plugins: Adding ForceTorqueSensor Plugin
* Updated to Apache 2.0 license
* Merge pull request `#180 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/180>`_ from vrabaud/indigo-devel
  remove PCL dependency
* merging
* check deprecation of gazebo::Joint::SetAngle by SetPosition
* compatibility with gazebo 4.x
* Update changelogs for the upcoming release
* Fix build with gazebo4 and indigo
* Added Gaussian Noise generator
* publish organized pointcloud from openni plugin
* Changed measurement direction to "parent to child"
* gazebo_plugin: Added updateRate parameter to the gazebo_ros_imu plugin
* gazebo_plugins: Adding ForceTorqueSensor Plugin
* remove PCL dependency
* ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
  ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
* Contributors: John Hsu, Jonathan Bohren, Jose Luis Rivero, Nate Koenig, Ryohei Ueda, Vincent Rabaud, fsuarez6, gborque, John Binney

2.4.5 (2014-08-18)
------------------
* Replace SetAngle with SetPosition for gazebo 4 and up
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero, Steven Peters

2.4.4 (2014-07-18)
------------------
* Merge branch 'hydro-devel' into indigo-devel
* gazebo_ros_diff_drive gazebo_ros_tricycle_drive encoderSource option names updated
* gazebo_ros_diff_drive is now able to use the wheels rotation of the optometry or the gazebo ground truth based on the 'odometrySource' parameter
* simple linear controller for the tricycle_drive added
* second robot for testing in tricycle_drive_scenario.launch added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* BDS licenses header fixed and tricycle drive plugin added
* format patch of hsu applied
* Updated package.xml
* Fix repo names in package.xml's
* ros diff drive supports now an acceleration limit
* Pioneer model: Diff_drive torque reduced
* GPU Laser test example added
* fixed gpu_laser to work with workspaces
* hand_of_god: Adding hand-of-god plugin
  ros_force: Fixing error messages to refer to the right plugin
* Remove unneeded dependency on pcl_ros
* minor fixes on relative paths in xacro for pioneer robot
* gazebo test model pionneer 3dx updated with xacro path variables
* pioneer model update for the multi_robot_scenario
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fixed camera to work with workspaces
* fixed links related to changed name
* diff drive name changed to multi robot scenario
* working camera added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fix in pioneer xacro model for diff_drive
* Laser colour in rviz changed
* A test model for the ros_diff_drive ros_laser and joint_state_publisher added
* the ros_laser checkes now for the model name and adds it als prefix
* joint velocity fixed using radius instead of diameter
* ROS_INFO on laser plugin added to see if it starts
* fetched with upstream
* gazebo_ros_diff_drive was enhanced to publish the wheels tf or the wheels joint state depending on two additinal xml options <publishWheelTF> <publishWheelJointState>
* Gazebo ROS joint state publisher added
* Contributors: Dave Coleman, John Hsu, Jon Binney, Jonathan Bohren, Markus Bader, Steven Peters

2.4.3 (2014-05-12)
------------------
* gazebo_plugins: add run-time dependency on gazebo_ros
* Merge pull request `#176 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/176>`_ from ros-simulation/issue_175
  Fix `#175 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/175>`_: dynamic reconfigure dependency error
* Remove unneeded dependency on pcl_ros
* Fix `#175 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/175>`_: dynamic reconfigure dependency error
* Contributors: Steven Peters

2.4.2 (2014-03-27)
------------------
* merging from hydro-devel
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.4.1 (2013-11-13)
------------------

2.3.5 (2014-03-26)
------------------
* update test world for block laser
* this corrects the right orientation of the laser scan and improves on comparison between 2 double numbers
* Initialize ``depth_image_connect_count_`` in openni_kinect plugin
* multicamera bad namespace. Fixes `#161 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/161>`_
  There was a race condition between GazeboRosCameraUtils::LoadThread
  creating the ros::NodeHandle and GazeboRosCameraUtils::Load
  suffixing the camera name in the namespace
* Use function for accessing scene node in gazebo_ros_video
* readded the trailing whitespace for cleaner diff
* the parent sensor in gazebo seems not to be active
* Contributors: Dejan Pangercic, Ian Chen, John Hsu, Jordi Pages, Toni Oliver, Ugo Cupcic

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* Simplify ``gazebo_plugins/CMakeLists.txt``
  Replace ``cxx_flags`` and ``ld_flags`` variables with simpler cmake macros
  and eliminate unnecessary references to ``SDFormat_LIBRARIES``, since
  they are already part of ``GAZEBO_LIBRARIES``.
* Put some cmake lists on multiple lines to improve readability.
* Add dependencies on dynamic reconfigure files
  Occasionally the build can fail due to some targets having an
  undeclared dependency on automatically generated dynamic
  reconfigure files (GazeboRosCameraConfig.h for example). This
  commit declares several of those dependencies.

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* gazebo_plugins: use shared pointers for variables shared among cameras
  It is not allowed to construct a shared_ptr from a pointer to a member
  variable.
* gazebo_plugins: moved initialization of shared_ptr members of
  GazeboRosCameraUtils to `GazeboRosCameraUtils::Load()`
  This fixes segfaults in gazebo_ros_depth_camera and
  gazebo_ros_openni_kinect as the pointers have not been initialized
  there.
* Use `RenderingIFace.hh`

2.3.2 (2013-09-19)
------------------
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  `include <physics/physics.hh>` will not be valid
  `include <gazebo/physics/physics.hh>` must be done instead.
* Merge branch 'hydro-devel' of `gazebo_ros_pkgs <github.com:ros-simulation/gazebo_ros_pkgs>`_ into synchronize_with_drcsim_plugins
* change includes to use brackets in headers for export
* per pull request comments
* Changed resolution for searchParam.
* Don't forget to delete the node!
* Removed info message on robot namespace.
* Retreive the tf prefix from the robot node.
* synchronize with drcsim plugins

2.3.1 (2013-08-27)
------------------
* Remove direct dependency on pcl, rely on the transitive dependency from pcl_ros
* Cleaned up template, fixes for header files

2.3.0 (2013-08-12)
------------------
* enable image generation when pointcloud is requested, as the generated image is used by the pointcloud
* gazebo_plugins: replace deprecated boost function
  This is related to this `gazebo issue #581 <https://bitbucket.org/osrf/gazebo/issue/581/boost-shared_-_cast-are-deprecated-removed>`_
* gazebo_plugins: fix linkedit issues
  Note: other linkedit errors were fixed upstream
  in gazebo
* gazebo_ros_openni_kinect plugin: adds publishing of the camera info
  again (fixes `#95 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/95>`_)
* Merge pull request `#90 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/90>`_ from piyushk/add_model_controller
  added a simple model controller plugin that uses a twist message
* renamed plugin from model controller to planar move
* prevents dynamic_reconfigure from overwritting update rate param on start-up
* removed anonymizer from include guard
* fixed odometry publication for model controller plugin
* added a simple model controller plugin that uses a twist message to control models

2.2.1 (2013-07-29)
------------------
* Added prosilica plugin to install TARGETS

2.2.0 (2013-07-29)
------------------
* Switched to pcl_conversions instead of using compiler flags for Hydro/Groovy PCL support
* fixed node intialization conflict between gzserver and gzclient. better adherance to gazebo style guidelines
* Fixed template
* removed ros initialization from plugins
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information about the SDFormat package.
* ROS Video Plugin for Gazebo - allows displaying an image stream in an OGRE texture inside gazebo. Also provides a fix for `#85 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/85>`_.
* patch a fix for prosilica plugin (startup race condition where `rosnode_` might still be NULL).
* Added explanation of new dependency in gazebo_ros_pkgs
* switch Prosilica camera from type depth to regular camera (as depth data were not used).
* migrating prosilica plugin from pr2_gazebo_plugins
* Removed tbb because it was a temporary dependency for a Gazebo bug
* SDF.hh --> sdf.hh
* Added PCL to package.xml

2.1.5 (2013-07-18)
------------------
* Include <sdf/sdf.hh> instead of <sdf/SDF.hh>
  The sdformat package recently changed the name of an sdf header
  file from SDF.hh to SDFImpl.hh; this change will use the lower-case
  header file which should work with old and new versions of sdformat
  or gazebo.

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------
* temporarily add tbb as a work around for `#74 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/74>`_

2.1.2 (2013-07-12)
------------------
* Fixed compatibility with new PCL 1.7.0
* Tweak to make SDFConfig.cmake
* Re-enabled dynamic reconfigure for camera utils - had been removed for Atlas
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Removed SVN references
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Small deprecated warning
* Fixed errors and deprecation warnings from Gazebo 1.9 and the sdformat split
* Source code formatting.
* Merge pull request `#59 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/59>`_ from ros-simulation/CMake_Tweak
  Added dependency to prevent missing msg header, cleaned up CMakeLists
* export diff drive and skid steer for other catkin packages
* install diff_drive and skid_steer plugins
* Added dependency to prevent missing msg header, cleaned up CMakeLists
* Added ability to switch off publishing TF.

2.1.0 (2013-06-27)
------------------
* gazebo_plugins: always use gazebo/ path prefix in include directives
* gazebo_plugins: call Advertise() directly after initialization has
  completed in gazebo_ros_openni_kinect and gazebo_ros_depth_camera
  plugins, as the sensor will never be activated otherwise
* Merge pull request `#41 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/41>`_ from ZdenekM/hydro-devel
  Added skid steering plugin (modified diff drive plugin).
* Merge pull request `#35 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/35>`_ from meyerj/fix_include_directory_installation_target
  Header files of packages gazebo_ros and gazebo_plugins are installed to the wrong location
* Rotation fixed.
* Skid steering drive plugin.
* gazebo_plugins: added missing initialization of `GazeboRosDepthCamera::advertised_`
* gazebo_plugins: fixed depth and openni kinect camera plugin segfaults
* gazebo_plugins: terminate the service thread properly on destruction of a PubMutliQueue object without shuting down ros
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* fix for terminating the `service_thread_` in PubQueue.h
* added differential drive plugin to gazebo plugins

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Check camera util is initialized before publishing - fix from Atlas

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Created tests for various spawning methods
* Added debug info to shutdown
* Fixed gazebo includes to be in <gazebo/...> format
* Cleaned up file, addded debug info
* Merge branch 'groovy-devel' into plugin_updates
* Merged changes from Atlas ROS plugins, cleaned up headers
* Merged changes from Atlas ROS plugins, cleaned up headers
* fix curved laser issue
* Combining Atlas code with old gazebo_plugins
* Combining Atlas code with old gazebo_plugins
* Small fixes per ffurrer's code review
* Added the robot namespace to the tf prefix.
  The tf_prefix param is published under the robot namespace and not the
  robotnamespace/camera node which makes it non-local we have to use the
  robot namespace to get it otherwise it is empty.
* findreplace ConnectWorldUpdateStart ConnectWorldUpdateBegin
* Fixed deprecated function calls in gazebo_plugins
* Deprecated warnings fixes
* Removed the two plugin tests that are deprecated
* Removed abandoned plugin tests
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
