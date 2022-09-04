# ovis_robotiq_gripper

## Description and source

This project is made to be used by Club Capra's other repository, [ovis](https://github.com/clubcapra/ovis). It allows the usage of a robotiq 2 finger gripper through ethercat. The repository is a lighter fork of Robotiq's ros-industrial [code](https://github.com/ros-industrial/robotiq).

## Usage

Using the ovis_robotiq_gripper node, you can alternate the gripper from opened to closed by sending a OvisGripperPosition message with a value of 2 in the position field on the /ovis/gripper/position_goal topic.

The intended usage is to have the command sent by our [UI](https://github.com/clubcapra/capra_web_ui) using the B button when in the arm control state.