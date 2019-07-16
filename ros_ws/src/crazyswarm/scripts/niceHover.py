#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.5
tspan = 2.0

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=tspan)
    timeHelper.sleep(tspan+.5)
    '''for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)'''

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=tspan)
    timeHelper.sleep(tspan+.1)
