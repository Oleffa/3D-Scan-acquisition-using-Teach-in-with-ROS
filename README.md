# 3D-Scan Acquisition using Teach-in with a mobile Robot and ROS

## Introduction

This Repository contains all packages created and modified during the course of my bachelor thesis at the chair for Robotics and Telematics of the Julius-Maximilians-Universität Würzburg.
The functionalities have been optimized and presented at the conference "Oldenburger 3D Tage 2016" in Oldenburg (https://www.jade-hs.de/unsere-hochschule/fachbereiche/bauwesen-geoinformation-gesundheitstechnologie/geoinformation/veranstaltungen/oldenburger-3d-tage/). More information can be found in the included german PDF version of my thesis or the article published in "Photogrammetrie - Laserscanning - Optische 3D-Messtechnik" (ISBN 978-3-87907-625-3).

## Features
- Controling a mobile robot using a different kinds of USB-gamepads
- Manual Path recording and path following of a recorded path (automatic or manual)
- Generating of 2D-maps of an environment + Localization in the 2D-map using simultaneous localization and mapping (SLAM). Either by using the the amcl package of ROS or ohm_tsd_slam (http://wiki.ros.org/ohm_tsd_slam)
- Manual and automatic 3D-scanning using a RIEGL VZ-400 Laserscanner
- Waypoint saving on a path as positions for automated 3D-scans

## Process
  1. Teaching in a path and waypoints to the robot and generating a 2D-Map of the environment using amcl or ohm_tsd_slam
  2. Automatic repetition of the path and execution of 3D-scans at waypoints
  3. Get a coffee and watch your robot do the work
  4. Processing the scan data
  
## Installation
  - If the ohm-tsd-slam (used for localization) package is to be used you need the obviously library. For more information visit: https://github.com/autonohm/obviously
  - For use with the ros amcl package just do not build the ohm_tsd_slam package
  
## Generation of 3D-Map
  - In this project the 3D Toolkit (3DTK: http://slam6d.sourceforge.net/index.html) was used to process the single scans into one 3D-Point cloud

## Acknowledgements
Thanks to Prof. Dr. Nüchter to supervise my bachelor thesis and to MSc. Dorit Borrmann for helping me with problems and optimizing and presenting the project at the "Oldenburger 3D Tage"

