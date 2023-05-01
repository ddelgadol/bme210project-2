#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  DemoIK.py - York Hack Space May 2014
#  Simple demo of meArm library to walk through some points defined in Cartesian coordinates

import meArm


arm = meArm.meArm()
arm.begin(0,0x6f) # block address of motor controller

#arm.openGripper()
arm.closeGripper()
arm.openGripper()
#arm.paropenGripper()
#arm.parcloseGripper()
arm.closeGripper()
#arm.openGripper()

#arm.gotoPoint(   0, 150,   0)#moves 150 blocks to the front
#arm.gotoPoint(   0, 150,  50)#moves 50 blocks up
#arm.gotoPoint(   0, 150, 100)#moves 50 additional blocks up
#arm.gotoPoint(   0, 150,  50)#moves 50 blocks down
#arm.gotoPoint(   0, 200,  50)#moves 50 additional blocks to the front
#arm.gotoPoint(   0, 100,  50)#moves 100 blocks back
#arm.gotoPoint(   0,  80,  50)#moves 20 blocks back
#arm.gotoPoint(   0, 150,  50)#moves 70 blocks forward
#arm.gotoPoint(-100, 150,  50)#moves 100 blocks to the left
#arm.gotoPoint( 100, 150,  50)#moves 200 blocks to the right
#arm.gotoPoint(   0, 150,  50)#moves 100 blocks to the left




