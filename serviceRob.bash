#! /bin/sh

rosservice call /ia_slam1/starter "demand: true" 
rosservice call /ia_slam2/starter "demand: true" 
rosservice call /ia_slam3/starter "demand: true" 

