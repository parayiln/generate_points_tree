import pybullet as p
import time
import math
import pybullet_data
import numpy as np


# useGui = True
#
# if (useGui):
#   p.connect(p.GUI)
# else:
#   p.connect(p.DIRECT)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
def ray_trace(usegui,center_pt,len_cylinder, orient, id, ray_count):
    useGui=usegui

    p.resetBasePositionAndOrientation(id,center_pt,orient)
    quater=p.getBasePositionAndOrientation(id)
    # point_center=[center_pt[0]-1.0, center_pt[1]-1, center_pt[2]+1]
    point_center=quater[0]
    # oreint_euler =p.getMatrixFromQuaternion(quater[1])
    rayFrom = []
    rayTo = []
    rayIds = []

    rayLen = 10

    rayHitColor = [1, 0, 0]
    rayMissColor = [0, 1, 0]

    # print("lllllll",oreint_euler)
    replaceLines = True
    theta2 = np.arange(0, 2*math.pi, math.pi/180, dtype=float)
    theta = np.arange(-math.pi/12, math.pi/12, math.pi/90, dtype=float)

    a=0
    count=ray_count
    cylinder_len=.1
    for i in range(len(theta)):
        for j in range(len(theta2)):
          count=count+1
          new=p.getQuaternionFromEuler([1.51,0,0])
          t=p.multiplyTransforms([0,0,0],new,[0,0,0],quater[1])
          trans=p.multiplyTransforms(point_center,t[1],[cylinder_len* math.sin(theta2[j])*math.cos(theta[i]),cylinder_len* math.sin(theta[i])*math.sin(theta2[j]),cylinder_len*math.cos(theta2[j])],[1,0,0,1])
          pt_trans=p.multiplyTransforms(point_center,t[1],[rayLen* math.sin(theta2[j])*math.cos(theta[i]),rayLen * math.sin(theta[i])*math.sin(theta2[j]),rayLen*math.cos(theta2[j])],[1,0,0,1])
          rayFrom.append([
              trans[0][0],
              trans[0][1],
              trans[0][2]
          ])
          rayTo.append([
              pt_trans[0][0],
              pt_trans[0][1],
              pt_trans[0][2]
          ])
          if (replaceLines):
            rayIds.append(p.addUserDebugLine(rayFrom[a], rayTo[a], rayMissColor))
          else:
            rayIds.append(-1)
          a=a+1


    if (not useGui):
      timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "rayCastBench.json")

    numSteps = 10
    if (useGui):
      # numSteps = 327680
        numSteps = count

    for i in range(numSteps):
      p.stepSimulation()
      for j in range(8):
        results = p.rayTestBatch(rayFrom, rayTo, j + 1,parentLinkIndex=id)
      #for i in range (10):
      #	p.removeAllUserDebugItems()

      if (useGui):
        if (not replaceLines):
          p.removeAllUserDebugItems()

        for i in range(len(rayTo)):
          print("hitt")
          hitObjectUid = results[i][0]

          if (hitObjectUid < 0):
            hitPosition = [0, 0, 0]
            p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
          else:
            hitPosition = results[i][3]
            p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])

        # time.sleep(1./240.)

      if (not useGui):
        p.stopStateLogging(timingLog)
      return count
