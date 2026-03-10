# Module HelloMotion

A Viam module that provides arm motion commands for waving and picking up objects detected by a vision service.

## Models

This module provides the following model(s):

- [`viamdemo:HelloMotion:hellomotion`](viamdemo_HelloMotion_hellomotion.md) - A generic service that controls an arm and gripper to wave or pick up the largest detected object.

## Dependencies

This module requires the following resources to be configured on your machine:

- An **arm** component
- A **gripper** component
- A **vision** service (e.g., [`viam:vision:obstacles-pointcloud`](https://github.com/viam-modules/obstacles-pointcloud))
- The built-in **motion** service
