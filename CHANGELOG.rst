^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package occupancy_grid_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.0.7 (2015-09-22)
------------------
* Merge pull request `#8 <https://github.com/clearpathrobotics/occupancy_grid_utils/issues/8>`_ from clearpathrobotics/raytrace_free_space
  Raytrace free space based on max_distance
* Update raytrace comment
* Raytrace free space based on max_distance
* Contributors: James Servos

0.0.6 (2015-08-30)
------------------
* Fix non-installed *.hpp headers
* Contributors: Enrique Fernandez

0.0.5 (2015-02-25)
------------------
* Changed OpenCV dependency to 'cv_bridge' to work on hydro and indigo
* Compile in Boost libraries
* Fix compilation, gcc doesn't like use of the optional vars
* Fix some cppcheck warnings
* Use boost::ref(), not just ref()
* Contributors: Alex Bencz, Jonathan Jekir, Mike Purvis, Siegfried-A. Gevatter Pujals

0.0.4 (2014-02-19)
------------------
* change libopencv-dev to opencv2, since libopencv-dev is not present in the rosdep database for precise
* 0.0.4
* update changelog
* fixed a dependency issue with opencv
* Contributors: y22ma

0.0.3 (2014-02-16)
------------------
* remove dependency on SDL, which is terrible for CI build due to X server dependencies
* fix crlf for package.xml
* Contributors: y22ma

0.0.2 (2014-02-13)
------------------

0.0.1 (2014-02-13)
------------------
* Initial Hydro release catkinized from https://code.ros.org/svn/ros-pkg/stacks/graph_mapping/trunk/occupancy_grid_utils r40053.
* Contributors: Andreas Wachaja, Bhaskara Marthi, Mac Mason, Yan Ma
