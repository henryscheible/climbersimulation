import logging

import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
logging.fatal(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0]

startOrientation = p.getQuaternionFromEuler([0, 0, 0])
hangarID = p.loadURDF("./examples/hangar/hangar.urdf", [0, 0, 0], startOrientation, useFixedBase=True)
robotID = p.loadURDF("./examples/slider-climber/robot.urdf", [0.5, -1, 1], p.getQuaternionFromEuler([0, 0, 3.14159]))
totalTime = 500

commands = [  # (time, joint, position)
    (1, 1, -0.5334),
    (2, 0, -0.3),
    (3, 0, 0.3),
    (4, 1, -0.3),
    (5, 0, -0.3),
    (6, 1, 0.5334),
    (7, 0, -0.4064),
    (8, 1, 0.25),
    (9, 0, -0.2),
    (10, 1, -0.5334),
    (11, 0, 0.3),
    (14, 1, -0.3),
    (15, 0, -0.3),
    (16, 1, 0.4),
    (17, 0, -0.4064),
    (16.5, 1, 0.25),
    (19, 0, 0),
    (20, 1, 0)
]

events = [None] * (240 * totalTime)

for command in commands:
    events[int(command[0] * 240)] = (command[1], command[2])

input()
for i, event in enumerate(events):
    p.stepSimulation()
    time.sleep(1. / 240.)
    if i % 240 == 0:
        logging.info("Time: " + str(i / 240) + " seconds")
    # pprint(
    #     p.getJointStates(bodyUniqueId=robotID,
    #                      jointIndices=[0, 1])
    # )
    if event is not None:
        logging.fatal("Running Event!")
        p.setJointMotorControl2(bodyUniqueId=robotID,
                                jointIndex=event[0],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=event[1],
                                positionGain=0.01)
