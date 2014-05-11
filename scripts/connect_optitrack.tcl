#!/usr/bin/env tclsh

package require genomix
genomix::connect
genomix1 rpath /home/hkhambha/openrobots/devel/lib/genom/ros/plugins/
genomix1 load optitrack
optitrack::connect {host marey host_port 1510 mcast 239.255.42.99 mcast_port 1511} 
