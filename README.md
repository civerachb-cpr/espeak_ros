espeak_ros
======================================

This ROS package is a simple wrapper for the espeak library, allowing
simple text-to-speech generation

This package is a complete rewrite of https://github.com/muhrix/espeak_ros,
removing the dynamic_reconfigure support in favour of a simple string topic
subscription and static rosparam configuration.

Building
------------------------

This package uses the standard `catkin_make` command to build.  Clone the repo
into the `src` folder of your workspace, install dependencies with `rosdep`,
and build:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace src
cd src
git clone https://github.com:civerachb-cpr/espeak_ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

The only non-standard ROS dependency is `libespeak-dev`.  On Melodic with
Ubuntu 18.04 this is resolvable by `rosdep`.  If your system can't evaluate
this dependency for some reason, you can install the library manually with

```bash
    sudo apt-get install libespeak-dev
```

Both `libespeak` and `espeak_node` also make use of `pthreads`.  This may
cause errors if you try using the node on Windows; so far it's been tested
on Ubuntu only.


Running
----------

To launch the node, either use the provided launch file, or include `espeak_node`
in another launch file:

```bash
roslaunch espeak_ros espeak_node.launch
```

or

```xml
<node name="espeak_node" pkg="espeak_ros" type="espeak_node">
  <rosparam command="load" file="$(find espeak_ros)/config/espeak.yaml" />
  <!-- To automatically subscribe to another topic you can use
  <remap from="voice_input" to="/some/other/topic" />
  -->
</node>
```

The `espeak_node` node simply subscribes to the `voice_input` topic.  Publishing
a `std_msgs/String` message to this topic will trigger the voice synthesis
automatically.

If your topic publishes at a rate faster than the `espeak_node` can process, older
messages will be dropped.  The node keeps a queue with a maximum length and uses
a worker thread to pull messages out of the queue and process them.

The node accepts the following parameters, defined in `espeak_ros/config/espeak.yaml`:

| Parameter  | Default Value | Description                                                |
| ---------- | ------------- | ---------------------------------------------------------- |
| `voice`    | `default`     | The espeak voice to use. e.g. `en-gb`, `en-us+f3`, `fr-fr` |
| `maxQueue` | `1`           | The maximum size of the message queue                      |
