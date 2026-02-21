#!/bin/bash

ros2 topic pub --once /figure_type std_msgs/msg/String "{data: line}"