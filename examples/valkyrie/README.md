# Valkyrie

This experimental directory does not adhere to the development practices that
apply elsewhere in Drake. In particular, for PRs that affect only this
directory, one feature review is sufficient; platform review is not required.

If build or test targets in this directory break, and the PR author or oncall
buildcop cannot trivially resolve them, a GitHub issue will be assigned to
the Valkyrie team. If the issue is not resolved within 24 hours, the author
or buildcop may disable the offending targets.

To compile this example
  $ cd drake
  $ bazel build //tools:drake_visualizer //examples/valkyrie/...

To run the visualizer:
  $ cd drake
  $ bazel-bin/tools/drake_visualizer &

To run the pd + feedforward controller:
  $ cd drake
  $ bazel-bin/examples/valkyrie/valkyrie_pd_ff_controller &

To run the simulation:
  $ cd drake
  $ bazel-bin/examples/valkyrie/valkyrie_simulation &

The visualizer needs to be started before the simulator.
The controller and simulator are not synchronized in any way, and they both
try to run as fast as possible.
For a bit more repeatability, start the controller first.

The controlled simulation eventually fails when the robot falls down, because
the simple controller does not account for sliding feet.

# Historical note

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation of Valkyrie you may use this
tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Valkyrie

# Box Code - Ethan Weber

- box_atlas
This code is usable and simple. In the box_atlas.h file, there are some user-modified variables. By editing the names of bodies that correspond to the correct alias groups file (in this case `atlas.alias_groups`), you can specify the string names by editing one of the private variable names. The program will use this specified data to publish the x,y,z position and velocities over the robot_state_t LCM type. It's called box_atlas because we only care about a few points on the robot. The center of mass (specified as the x,y,z values in the pose translation component) and the specified bodies.

- box
This file is not very clean, but it contains all the necessary code to create QP inputs and control any robot with a valid URDF file. The box.cc and box.h files are used with the box binary (`bazel run --config gurobi box`). This works to specify some controls, but I didn't finish this development for what was needed. It could easily be picked up for later use with the current drake controller.
