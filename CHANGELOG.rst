^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package espeak_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0
-----
* Complete rewrite
* Remove dynamic_reconfigure, use static rosparam file for configuration
* Use a simple ROS topic as the input method to the voice-synthesis
* Contributors: Chris Iverach-Brereton

0.0.1
-----
* Made changes to allow use of dynamic_reconfigure
* Added dynamic reconfigure settings
* Added dynamic reconfigure cfg file
* Updated gitignore
* Added ROS parameters to configure espeak library
* Added dependency to dynamic_reconfigure
* Added gitignore file to repo
* Fixed bug when passing length of string (it was speaking rubbish at the end of some strings)
* Updated README
* Corrected typos/mismatches in espeak lib variables
* Added author information to file and modified path to included header file
* Initial commit
* Contributors: Murilo FM
