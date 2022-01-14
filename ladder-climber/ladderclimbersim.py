import logging
import math

import pybullet as p
import pybullet_data
import time
from pprint import pprint

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
logging.fatal(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0]

startOrientation = p.getQuaternionFromEuler([0, 0, 0])
hangarID = p.loadURDF("../hangar/hangar.urdf", [0, 0, 0], startOrientation, useFixedBase=True)
robotID = p.loadURDF("./description/robot.urdf", [0.05, -1, 0], startOrientation)
totalTime = 8
p.resetJointState(bodyUniqueId=robotID,
                  jointIndex=0,
                  targetValue=0.889,
                  targetVelocity=0)

commands = [  # (time, joint, position)
    (1, 1, -3),
    (2, 0, -0.3),
    (5, 1, 0.2),
    (6, 0, 0)
]

hook_angle_id = p.addUserDebugParameter("Passive Hook Angle", -3, 3, 0)
telescope_position_id = p.addUserDebugParameter("Telescope Position", -1, 5, 0)

events = [None] * (240 * totalTime)

for command in commands:
    events[int(command[0] * 240)] = (command[1], command[2])
#
# input()
# for i, event in enumerate(events):
#     p.stepSimulation()
#     time.sleep(1. / 240.)
#     if i % 240 == 0:
#         logging.info("Time: " + str(i / 240) + " seconds")
#     # pprint(
#     #     p.getJointStates(bodyUniqueId=robotID,
#     #                      jointIndices=[1, 2, 3, 4])
#     # )
#     if event is not None:
#         if event[0] == 0:
#             logging.fatal("Running Event!")
#             p.setJointMotorControl2(bodyUniqueId=robotID,
#                                     jointIndex=0,
#                                     controlMode=p.POSITION_CONTROL,
#                                     targetPosition=event[1],
#                                     positionGain=0.01)
#         else:
#             for i in range(3,5):
#                 logging.error("Updating Spring Loaded Hooks!")
#                 p.setJointMotorControl2(bodyUniqueId=robotID,
#                                         jointIndex=i,
#                                         controlMode=p.POSITION_CONTROL,
#                                         targetPosition=event[1],
#                                         positionGain=0.01)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
    angle = p.readUserDebugParameter(itemUniqueId=hook_angle_id)
    p.setJointMotorControl2(bodyUniqueId=robotID,
                            jointIndex=3,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            positionGain=0.01)
    p.setJointMotorControl2(bodyUniqueId=robotID,
                            jointIndex=4,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            positionGain=0.01)
    position = p.readUserDebugParameter(itemUniqueId=telescope_position_id)
    p.setJointMotorControl2(bodyUniqueId=robotID,
                            jointIndex=0,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=position,
                            positionGain=0.01)