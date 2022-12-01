#!/usr/bin/python3

import rospy
import math
# Importing pygame module
from time import sleep
import pygame
from pygame.locals import *
from proteus_hreye.msg import HREyeState

rospy.init_node("hreye_terminal_tester")

window = None
full_circle = 2 * math.pi
ring_centers = ([250,250], [750,250])
ring_width = 17.5
cam_radius = 40
outer_radius = 182.5
inner_radius= 102.5
large_sec = full_circle / 24
small_sec = full_circle / 16
light_centers = [[],[]]
light_radius = 10

light_colors = [[[200,200,200] for x in range(40)], [[200,200,200] for x in range(40)]]

last_msg = rospy.Time.now().secs * 1000
timeout = 100

def hreye_state_callback(msg):
    global light_colors, last_msg
    for k, rgba in enumerate(msg.state):
        if rgba.r == 0 and rgba.g == 0 and rgba.b == 0:
            light_colors[msg.hreye_index][k][0] = 200
            light_colors[msg.hreye_index][k][1] = 200 
            light_colors[msg.hreye_index][k][2] = 200 
        else:    
            light_colors[msg.hreye_index][k][0] = rgba.r * rgba.a
            light_colors[msg.hreye_index][k][1] = rgba.g * rgba.a 
            light_colors[msg.hreye_index][k][2] = rgba.b * rgba.a

    last_msg = rospy.Time.now().secs * 1000

if __name__ == "__main__":
    # initiate pygame and give permission
    # to use pygame's functionality.
    pygame.init()

    # create the display surface object
    # of specific dimension.
    window = pygame.display.set_mode((1000, 500))

    # Fill the screen with white color
    window.fill((255,255,255))

    #Drawing Left eye (non-dynamic parts.)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[0],cam_radius, 0)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[0],inner_radius + ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[0],inner_radius - ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[0],outer_radius + ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[0],outer_radius - ring_width, 5)

    #Same thing, but right side.
    pygame.draw.circle(window, (0, 0, 0),ring_centers[1], cam_radius, 0)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[1],inner_radius + ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[1],inner_radius - ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[1],outer_radius + ring_width, 5)
    pygame.draw.circle(window, (0, 0, 0),ring_centers[1],outer_radius - ring_width, 5)

    for k in range(2):
        ring_origin = ring_centers[k]
        # Now it is time to draw the lights. 
        light_spread = full_circle / 24
        for i in range(24):
            light_angle = 0 + (i *(light_spread)) - math.pi/2 # The final term is a bit of a kludge to fix indexing.
            x = (outer_radius * math.cos(light_angle)) + ring_origin[0]
            y = (outer_radius * math.sin(light_angle)) + ring_origin[1]
            light_centers[k].append([x,y])

        light_spread = full_circle / 16
        for i in range(16):
            light_angle = full_circle - (i *(light_spread)) - math.pi/2 # The final term is a bit of a kludge to fix indexing.
            x = (inner_radius * math.cos(light_angle)) + ring_origin[0]
            y = (inner_radius * math.sin(light_angle)) + ring_origin[1]
            light_centers[k].append([x,y])

        for cp in light_centers[k]:
            pygame.draw.circle(window, (0,0,0), cp, light_radius, 4)
            pygame.display.update()
            sleep(0.05)

    rospy.Subscriber("/loco/proteus/hreye/state", HREyeState, hreye_state_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Draws the surface object to the screen.

        for ring, center_list in enumerate(light_centers):
            for k, cp in enumerate(center_list):
                # print("Drawing {} at {}".format(light_colors[ring][k], cp))
                pygame.draw.circle(window, light_colors[ring][k], cp, light_radius-4, 0)

        if (rospy.Time.now().secs * 1000) - last_msg > timeout:
            light_colors = [[[200,200,200] for x in range(40)], [[200,200,200] for x in range(40)]]

        pygame.display.update()
        rate.sleep()

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: 
                pygame.quit()
