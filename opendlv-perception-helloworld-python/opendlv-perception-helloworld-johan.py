#!/usr/bin/env python3

# Copyright (C) 2018 Christian Berger
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# sysv_ipc is needed to access the shared memory where the camera image is present.
import sysv_ipc
# numpy and cv2 are needed to access, modify, or display the pixels
import numpy as np
import cv2
# OD4Session is needed to send and receive messages
import OD4Session
# Import the OpenDLV Standard Message Set.
import opendlv_standard_message_set_v0_9_10_pb2

from j_fun.j_funs import *

PRINT_STEER = False

################################################################################
# This dictionary contains all distance values to be filled by function onDistance(...).
distances = { "front": 0.0, "left": 0.0, "right": 0.0, "rear": 0.0 };
controls = {"steer": 0.0, "pedal": 0.0}

################################################################################
# This callback is triggered whenever there is a new distance reading coming in.
def onDistance(msg, senderStamp, timeStamps):
    # print ("Received distance; senderStamp= %s" % (str(senderStamp)))
    # print ("sent: %s, received: %s, sample time stamps: %s" % (str(timeStamps[0]), str(timeStamps[1]), str(timeStamps[2])))
    # print ("%s" % (msg))
    if senderStamp == 0:
        distances["front"] = msg.distance
    if senderStamp == 1:
        distances["left"] = msg.distance
    if senderStamp == 2:
        distances["rear"] = msg.distance
    if senderStamp == 3:
        distances["right"] = msg.distance


def onPedal(msg, senderStamp, timeStamps):
    controls["pedal"] = msg.distance
    print("pedal: " + str(msg.distance))


def onSteer(msg, senderStamp, timeStamps):
    controls["steer"] = msg.groundSteering
    if PRINT_STEER:
        print("steer: " + str(msg.groundSteering))


# Create a session to send and receive messages from a running OD4Session;
# Replay mode: CID = 253
# Live mode: CID = 112
# TODO: Change to CID 112 when this program is used on Kiwi.
session = OD4Session.OD4Session(cid=254)  #111
# Register a handler for a message; the following example is listening
# for messageID 1039 which represents opendlv.proxy.DistanceReading.
# Cf. here: https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
messageIDDistanceReading = 1039
messageIDSteerReading = 1090

session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_DistanceReading)
session.registerMessageCallback(messageIDSteerReading, onSteer, opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_GroundSteeringRequest)
# Connect to the network session.
session.connect()

################################################################################
# The following lines connect to the camera frame that resides in shared memory.
# This name must match with the name used in the h264-decoder-viewer.yml file.
name = "/tmp/img.argb"  # "/tmp/video0.argb"
# Obtain the keys for the shared memory and semaphores.
keySharedMemory = sysv_ipc.ftok(name, 1, True)
keySemMutex = sysv_ipc.ftok(name, 2, True)
keySemCondition = sysv_ipc.ftok(name, 3, True)
# Instantiate the SharedMemory and Semaphore objects.
shm = sysv_ipc.SharedMemory(keySharedMemory)
mutex = sysv_ipc.Semaphore(keySemCondition)
cond = sysv_ipc.Semaphore(keySemCondition)

################################################################################
# Main loop to process the next image frame coming in.
iii = 0
while True:
    # Wait for next notification.
    cond.Z()
    # print ("Received new frame.")

    # Lock access to shared memory.
    mutex.acquire()
    # Attach to shared memory.
    shm.attach()
    # Read shared memory into own buffer.
    buf = shm.read()
    # Detach to shared memory.
    shm.detach()
    # Unlock access to shared memory.
    mutex.release()

    # Turn buf into img array (1280 * 720 * 4 bytes (ARGB)) to be used with OpenCV.
    img = np.frombuffer(buf, np.uint8).reshape(720, 1280, 4)
    img = img[360:640, :]  # [0:280,  0:1280]
    Y_WEIGHTING = np.linspace(0.1, 1.0, img.shape[0])

    y_lo =219
    y_hi = 100
    x_mid = 640
    x_temp = 470
    HEIGHT = 100

    img2 = img
    ############################################################################
    # TODO: Add some image processing logic here.

    # Invert colors
    # img = cv2.bitwise_not(img)

    # img = cv2.Canny(img, 50, 150)  # edges
    img_blue, img_yellow = get_cones_imgs(img)

    _, thresh = cv2.threshold(img_blue, 127, 255, 1)  # blue
    contours_b, xs_center_b, ys_center_b = get_contours(thresh, type='b')

    ret, thresh = cv2.threshold(img_yellow, 127, 255, 1)  # blue
    contours_y, xs_center_y, ys_center_y = get_contours(thresh, type='y')

    center_mid_point_b = None
    if len(xs_center_b) > 0 and len(ys_center_b) > 0:
        center_mid_point_b = weighted_mean_centers(Y_WEIGHTING, xs_center_b, ys_center_b)

    center_mid_point_y = None
    if len(xs_center_y) > 0 and len(ys_center_y) > 0:
        center_mid_point_y = weighted_mean_centers(Y_WEIGHTING, xs_center_y, ys_center_y)

    if iii >= 30:
        gg = 5

    drawing = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    draw_s(img, contours_b, color=(255, 0, 0))
    draw_s(img, contours_y, color=(0, 255, 255))

    # # # STEERING VIEW OF OLD =============
    sign = -1
    if controls["steer"] < 0.0:
        sign = 1

    hyp = HEIGHT / np.cos(controls["steer"])
    opp = int(sign * np.sqrt(hyp**2 - HEIGHT **2))
    # print("opp: " + str(opp))

    cv2.line(img, pt1=[x_mid, y_lo], pt2=[x_mid + opp, y_hi], color=(0, 0, 255), thickness=5)
    # # ==============================================

    if center_mid_point_b is not None:
        cv2.circle(img, center_mid_point_b, radius=20, color=(255, 0, 0), thickness=10)

    if center_mid_point_y is not None:
        cv2.circle(img, center_mid_point_y, radius=20, color=(0, 255, 255), thickness=10)

    print("C b: " + str(len(contours_b['contours_poly'])) +
          "   C y: " + str(len(contours_y['contours_poly'])) +
          "    center: " + str(center_mid_point_b))
    cv2.imshow("drawing", img)

    cv2.waitKey(2)

    ############################################################################
    # Example: Accessing the distance readings.
    # print ("Front = %s" % (str(distances["front"])))
    # print ("Left = %s" % (str(distances["left"])))
    # print ("Right = %s" % (str(distances["right"])))
    # print ("Rear = %s" % (str(distances["rear"])))

    ############################################################################
    # Example for creating and sending a message to other microservices; can
    # be removed when not needed.
    angleReading = opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_AngleReading()
    # angleReading.angle = 123.45

    # 1038 is the message ID for opendlv.proxy.AngleReading
    # session.send(1038, angleReading.SerializeToString());

    ############################################################################
    # Steering and acceleration/decelration.
    #
    # Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
    # Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
    #groundSteeringRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_GroundSteeringRequest()
    #groundSteeringRequest.groundSteering = 0
    #session.send(1090, groundSteeringRequest.SerializeToString());

    # Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
    # Be careful!
    #pedalPositionRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_PedalPositionRequest()
    #pedalPositionRequest.position = 0
    #session.send(1086, pedalPositionRequest.SerializeToString());

    iii += 1