####################################################################
#
#  tmr_mod - timer device driver (for linux kernel with HZ = 125)
#
# Allows to measure time intervals with a discrete of 8 and 1000 milliseconds
#
#####################################################################

## Description

A simple linux kernel device driver

## Package files:

* tmr.c - source code of kernel device driver

* Makefile - make file (example compilation scenario)

* README.md

* drv.sh - load/unload module script

## Required:
```
linux kernel headers
```

## Compilation and installation
```
make
sudo make install
```

## Load/remove driver:
```
load  : sudo ./drv.sh load
unload: sudo ./drv.sh unload
```

## Using:

### From the user level functions are supported:
* open - open device
* close - close device
* read - reading time stamps
* write - set time stamp to zero - reset timer


