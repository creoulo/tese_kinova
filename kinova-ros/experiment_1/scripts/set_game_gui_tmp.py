#! /usr/bin/env python
"""Plays tic-tac-toe with minimax and alpha-beta algorithm with python gui"""

import rospy
import argparse
import os
import numpy as np
import Tkinter as tk
import tkSimpleDialog as simpledialog
import tkMessageBox as messagebox
import experiment_1_msgs.srv

from std_msgs.msg import *
from geometry_msgs.msg import *
from math import *
from copy import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from tf.transformations import *
from Tkinter import *


#poses_red = []
#poses_blue = []
#red_model_name = ['red_cylinder','red_cylinder_clone', 'red_cylinder_clone_0', 'red_cylinder_clone_1', 'red_cylinder_clone_1_clone']#for jaco tic_tac_toe
red_model_name = ['red_cylinder','red_cylinder_0', 'red_cylinder_1', 'red_cylinder_2', 'red_cylinder_3']#for jaco tic_tac_toe_kinect
#blue_model_name = ['blue_cylinder','blue_cylinder_clone', 'blue_cylinder_clone_clone', 'blue_cylinder_clone_clone_0', 'blue_cylinder_clone_clone_1']#for jaco tic_tac_toe
blue_model_name = ['blue_cylinder','blue_cylinder_0', 'blue_cylinder_1', 'blue_cylinder_2', 'blue_cylinder_3']#for jaco tic_tac_toe_kinect
game_state = ["","","","","","","","",""]
h = str(0.11) #height of the pieces needed to obtain the moves for the robot
win = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(1,5,8),(0,4,8),(2,4,6)]
global r, c

main = tk.Tk()
main.title('Tic-Tac-Toe')

'''
def getPieces():
    #gets the initial position of the pieces relative to the world
    #Red
    pose = (-0.302826, -0.139972)
    poses_red.append(pose)
    pose = (-0.299920, -0.239980)
    poses_red.append(pose)
    pose = (-0.300678, -0.339974)
    poses_red.append(pose)
    pose = (-0.300676, -0.442596)
    poses_red.append(pose)
    pose = (-0.301295, -0.540520)
    poses_red.append(pose)

    #Blue
    pose = (-0.4, -0.04)
    poses_blue.append(pose)
    pose = (-0.4, -0.14)
    poses_blue.append(pose)
    pose = (-0.4, -0.24)
    poses_blue.append(pose)
    pose = (-0.4, -0.34)
    poses_blue.append(pose)
    pose = (-0.4, -0.44)
    poses_blue.append(pose)

def getPosBoard(r,c):
    #gets the position of the board given the row and collumn relative to the world
    x = 0
    y = 0
    p_update = 9
    if r == 1:
        if c == 1:
            x = 0.098700
            y = -0.498752
            p_update = 0
        elif c == 2:
            x = -0.005644
            y = -0.498752
            p_update = 1
        elif c == 3:
            x = -0.105557
            y = -0.498752
            p_update = 2
    elif r == 2:
        if c == 1:
            x = 0.098700
            y = -0.406524
            p_update = 3
        elif c == 2:
            x = -0.005644
            y = -0.406524
            p_update = 4
        elif c == 3:
            x = -0.105557
            y = -0.406524
            p_update = 5
    elif r == 3:
        if c == 1:
            x = 0.098700
            y = -0.308745
            p_update = 6
        elif c == 2:
            x = -0.005644
            y = -0.308745
            p_update = 7
        elif c == 3:
            x = -0.105557
            y = -0.308745
            p_update = 8

    return x, y, p_update
'''
def validateMove(r, c):
    x, y, p_update = getPosBoard(r,c)
    if p_update < 0 or p_update > 9:
        return False
    if game_state[p_update]=="":
        return True
    else:
        return False

def gameStateUpdate(p_update, color):
    #updates array of plays
    game_state[p_update] = color

def endGame():
    #verifies state of the game
    #Four states: win red, win blue, tie, nothing(as in not over yet)
    for position in win:
        a, b, c = position
        if game_state[a] == game_state[b] and game_state[b] == game_state[c] and game_state[a] != "":
            if game_state[a] == "red":
                return "WIN RED"
            else:
                return "WIN BLUE"

    res = all(board != "" for board in game_state)
    if res == True:
        return "TIE"

    return "NOTHING"

def moveInGazebo(x, y, color):
    #calls gazebo service to place red pieces
    #receives position to be placed
    state_msg = ModelState()
    if color=="red":
        state_msg.model_name = red_model_name.pop(0)
    elif color=="blue":
        state_msg.model_name = blue_model_name.pop(0)
    state_msg.reference_frame = 'world'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0.20
    q = quaternion_from_euler(0, -pi, 0)
    state_msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def transformPos(pos):
    #transfroms position of game state array into row collumn in board
    r, c = 0, 0
    if pos == 0: r, c = 1, 1
    elif pos == 1: r, c = 1, 2
    elif pos == 2: r, c = 1, 3
    elif pos == 3: r, c = 2, 1
    elif pos == 4: r, c = 2, 2
    elif pos == 5: r, c = 2, 3
    elif pos == 6: r, c = 3, 1
    elif pos == 7: r, c = 3, 2
    elif pos == 8: r, c = 3, 3
    return r, c

#minimax algorithm
def maximization(depth, alpha, beta):
    #Robot play is maximized
    #Robot plays blue
    max_value = -2
    position = 9
    end_game = endGame()

    if end_game == "WIN RED":
        return -1, 0, 0
    elif end_game == "WIN BLUE":
        return 1, 0, 0
    elif end_game == "TIE":
        return 0, 0, 0
    elif depth == 0:
        return -1, 0, 0

    for pos in range(9):
        if game_state[pos] == "":
            game_state[pos] = "blue"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = ""

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value

    r, c = transformPos(position)
    return max_value, r, c

def minimization(depth, alpha, beta):
    #Human play is minimized
    #Human plays red
    min_value = 2
    position = 9
    end_game = endGame()

    if end_game == "WIN RED":
        return -1, 0, 0
    elif end_game == "WIN BLUE":
        return 1, 0, 0
    elif end_game == "TIE":
        return 0, 0, 0

    for pos in range(9):
        if game_state[pos] == "":
            game_state[pos] = "red"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = ""

            if min_value <= alpha:
                r, c = transformPos(position)
                return min_value, r, c
            if min_value < alpha:
                beta = min_value

    r, c = transformPos(position)
    return min_value, r, c

def get_button( r, c):
    if r - 1 == 0:
        if c - 1 == 0:
            return b1
        elif c - 1 == 1:
            return b2
        elif c - 1 == 2:
            return b3
    elif r - 1 == 1:
        if c - 1 == 0:
            return b4
        elif c - 1 == 1:
            return b5
        elif c - 1 == 2:
            return b6
    elif r - 1 == 2:
        if c - 1 == 0:
            return b7
        elif c - 1 == 1:
            return b8
        elif c - 1 == 2:
            return b9

def disable_all_buttons():
    b1.config(state=DISABLED)
    b2.config(state=DISABLED)
    b3.config(state=DISABLED)
    b4.config(state=DISABLED)
    b5.config(state=DISABLED)
    b6.config(state=DISABLED)
    b7.config(state=DISABLED)
    b8.config(state=DISABLED)
    b9.config(state=DISABLED)

# Button function
def b_click(b,r,c):
    if b["text"] == " " and endGame() == "NOTHING":
        b["text"] = "RED" #human played
        rx, ry, update_board = getPosBoard(int(r), int(c)) # visualize in gazebo
        moveInGazebo(rx, ry, "red")
        gameStateUpdate(update_board, "red")

        #robot plays next if human did not won
        if endGame() == "NOTHING":
            #pick vars
            x, y = poses_blue.pop(0)
            x = str(x)
            y = str(y)
            #place vars
            max_value, r, c = maximization(level, -2, 2)
            rx, ry, update_board = getPosBoard(r, c)
            gameStateUpdate(update_board, "blue")
            #show where robot plays on grid
            b_b = get_button(r, c) #returns string with button
            b_b["text"] = "BLUE"
            #moveInGazebo(rx, ry, "blue") #move directly in gazeb without the arm
            rx = str(rx)
            ry = str(ry)
            #execute pick and place
            cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + h + ' ' + rx + ' ' + ry + ' ' + h
            os.system(cmd)
    elif b["text"] != " " and endGame() == "NOTHING":
        messagebox.showerror("Tic-Tac-Toe", "Position already occupied")
    if endGame() != "NOTHING":
        end_game = endGame()
        if end_game == "RED WINS":
            messagebox.showinfo("Tic-Tac-Toe", "RED wins")
        elif end_game == "BLUE WINS":
            messagebox.showinfo("Tic-Tac-Toe", "BLUE wins")
        else:
            messagebox.showinfo("Tic-Tac-Toe", "It's a TIE")
        disable_all_buttons()
        #reset simulation
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

#Start new Game for the gui
def new():
    global b1, b2, b3, b4, b5, b6, b7, b8, b9
    global level

    #ask for dificulty wanted
    level = simpledialog.askstring("Input", "Choose dificulty between 1(easiest) and 8(hardest):",parent=main)
    level = int(level)

    # Buttons for the grid
    b1 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b1,1,1))
    b2 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b2,1,2))
    b3 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b3,1,3))

    b4 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b4,2,1))
    b5 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b5,2,2))
    b6 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b6,2,3))

    b7 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b7,3,1))
    b8 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b8,3,2))
    b9 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b9,3,3))

    # Button in a grid
    b1.grid(row=0, column=0)
    b2.grid(row=0, column=1)
    b3.grid(row=0, column=2)

    b4.grid(row=1, column=0)
    b5.grid(row=1, column=1)
    b6.grid(row=1, column=2)

    b7.grid(row=2, column=0)
    b8.grid(row=2, column=1)
    b9.grid(row=2, column=2)

if __name__ == '__main__':
  try:
    #--------------------------------------------------------------------------
    rospy.init_node('gameplay')
    getPosBoard = rospy.ServiceProxy('PosBoard', experiment_1_msgs.srv.PosBoard )


    #Initialization------------------------------------------------------------
    #Define initial poses of pieces to be picked up by robot
    getPieces()

    #Gameplay-----------------------------------------------------------------
    #end_game = endGame()
    #color = "red" #RED pieces always start - red is human

    # Menu
    menu = Menu(main)
    main.config(menu=menu)
    # Options for Menu
    options_menu = Menu(menu, tearoff=False)
    menu.add_cascade(label="Options", menu=options_menu)
    options_menu.add_command(label="New Game", command=new)

    new()
    main.mainloop()


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
