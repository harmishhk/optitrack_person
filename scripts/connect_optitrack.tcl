#!/usr/bin/env tclsh

package require genomix
genomix::connect
genomix1 load optitrack
after 1000
optitrack::connect {host marey host_port 1510 mcast 255.255.255.255 mcast_port 1511}
