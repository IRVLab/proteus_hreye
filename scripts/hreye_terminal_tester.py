#!/usr/bin/python3

import rospy
# Importing pygame module
import pygame
from pygame.locals import *


if __name__ == "__main__":
    # initiate pygame and give permission
    # to use pygame's functionality.
    pygame.init()

    rospy.init_node("hreye_terminal_tester")

    # create the display surface object
    # of specific dimension.
    window = pygame.display.set_mode((1200, 500))

    # Fill the screen with white color
    window.fill((255, 255, 255))

    #Drawing Left eye (non-dynamic parts.)
    pygame.draw.circle(window, (0, 0, 0),[300, 250],40, 0)
    pygame.draw.circle(window, (0, 0, 0),[300, 250],100, 5)
    pygame.draw.circle(window, (0, 0, 0),[300, 250],135, 5)
    pygame.draw.circle(window, (0, 0, 0),[300, 250],200, 5)
    pygame.draw.circle(window, (0, 0, 0),[300, 250],165, 5)

    #Same thing, but right side.
    pygame.draw.circle(window, (0, 0, 0),[900, 250], 40, 0)
    pygame.draw.circle(window, (0, 0, 0),[900, 250],100, 5)
    pygame.draw.circle(window, (0, 0, 0),[900, 250],135, 5)
    pygame.draw.circle(window, (0, 0, 0),[900, 250],200, 5)
    pygame.draw.circle(window, (0, 0, 0),[900, 250],165, 5)

    # Now it is time to draw the lights. 
    pyg

    while not rospy.is_shutdown():

        # Draws the surface object to the screen.
        pygame.display.update()
