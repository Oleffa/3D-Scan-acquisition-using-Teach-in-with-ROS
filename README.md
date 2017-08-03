# 3D-Scan Acquisition using Teach-in with a mobile Robot and ROS

## Introduction

This Repository contains a runnable copy of the code i wrote and modified during the course of my bachelor thesis at the chair for Robotics and Telematics of the Julius-Maximilians-Universität Würzburg. The functionalities have been optimized and presented at the conference "Oldenburger 3D Tage 2016" in Oldenburg (https://www.jade-hs.de/unsere-hochschule/fachbereiche/bauwesen-geoinformation-gesundheitstechnologie/geoinformation/veranstaltungen/oldenburger-3d-tage/). More information can be found in the included german PDF version of my thesis or the article published in "Photogrammetrie - Laserscanning - Optische 3D-Messtechnik" (ISBN 978-3-87907-625-3).

## Features
- Controling a mobile robot using a Logitech Gamepad
- Manual Path recording and path following of a recorded path (automatic or manual)
- Generating of 2D-maps of an environment + Localization in the 2D-map using simultaneous localization and mapping (SLAM) and the amcl package of ROS
- Manual and automatic 3D-scanning using a RIEGL VZ-400 Laserscanner
- Saving waypoints on a path for automatic 3D-scans

## Process
  1. Teaching in a path and waypoints to the robot while generating a 2D-Map of the environment
  2. Automatic repetition of the path and execution of 3D-scans at waypoints
  3. Get a coffee and watch your robot do the work

## Acknowledgements
Thanks to Prof. Dr. Nüchter to supervise my bachelor thesis and to MSc. Dorit Borrmann for helping me with problems and optimizing and presenting the project at the "Oldenburger 3D Tage"

