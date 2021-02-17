#! /usr/bin/env python
"""Plays trafic lights board game with minimax and alpha-beta algorithm with python gui"""

import rospy
import argparse
import os
import numpy as np
import Tkinter as tk
import tkSimpleDialog as simpledialog
import tkMessageBox as messagebox
import experiment_1_msgs.srv
import experiment_1_msgs.msg
import time

from std_msgs.msg import *
from geometry_msgs.msg import *
from math import *
from copy import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from tf.transformations import *
from Tkinter import *
from experiment_1_msgs.srv import *
from experiment_1_msgs.msg import *

global board_coordinates, green_model_name, green_model_name_h, yellow_model_name, yellow_model_name_h, red_model_name, red_model_name_h
global poses_green, poses_yellow, poses_red
poses_green = List_2()
poses_yellow = List_2()
poses_red = List_2()
game_state = ["","","","","","","","","","","",""]
win = [(0,1,2),(1,2,3),(4,5,6),(5,6,7),(8,9,10),(9,10,11),(0,4,8),(1,5,9),(2,6,10),(3,7,11),(0,5,10),(1,6,11),(3,6,9),(2,5,8)]
last_move = "ROBOT"
green_model_name = ['green_cylinder_short','green_cylinder_short_clone', 'green_cylinder_short_clone_0', 'green_cylinder_short_clone_1', 'green_cylinder_short_clone_2']
yellow_model_name = ['yellow_cylinder_short','yellow_cylinder_short_clone', 'yellow_cylinder_short_clone_0', 'yellow_cylinder_short_clone_1', 'yellow_cylinder_short_clone_2']
red_model_name = ['red_cylinder_short','red_cylinder_short_clone', 'red_cylinder_short_clone_0', 'red_cylinder_short_clone_1', 'red_cylinder_short_clone_2']
green_model_name_h = ['green_cylinder_short_clone_3','green_cylinder_short_clone_4', 'green_cylinder_short_clone_5', 'green_cylinder_short_clone_6', 'green_cylinder_short_clone_7', 'green_cylinder_short_clone_8']
yellow_model_name_h = ['yellow_cylinder_short_clone_3','yellow_cylinder_short_clone_4', 'yellow_cylinder_short_clone_5', 'yellow_cylinder_short_clone_6', 'yellow_cylinder_short_clone_7']
red_model_name_h = ['red_cylinder_short_clone_3','red_cylinder_short_clone_4', 'red_cylinder_short_clone_5', 'red_cylinder_short_clone_6', 'red_cylinder_short_clone_7']

main = tk.Tk()
main.title('Traffic Lights')

def initServices():
    #Instanciate all the services
    rospy.Service(name="experiment_1_msgs/PosBoard", service_class=PosBoard,
                                                    handler=pos_board_handler)

#define all the handlers
def pos_board_handler( req):
    # given the row and collumn returns the position of the board relative to
    # the world and the position of the array for the game state
    global board_coordinates
    x = 0
    y = 0
    pos_update = 9
    if req.r == 1:
        if req.c == 1:
            x = board_coordinates[0].x
            y = board_coordinates[0].y
            pos_update = 0
        elif req.c == 2:
            x = board_coordinates[1].x
            y = board_coordinates[1].y
            pos_update = 1
        elif req.c == 3:
            x = board_coordinates[2].x
            y = board_coordinates[2].y
            pos_update = 2
        elif req.c == 4:
            x = board_coordinates[3].x
            y = board_coordinates[3].y
            pos_update = 3
    elif req.r == 2:
        if req.c == 1:
            x = board_coordinates[4].x
            y = board_coordinates[4].y
            pos_update = 4
        elif req.c == 2:
            x = board_coordinates[5].x
            y = board_coordinates[5].y
            pos_update = 5
        elif req.c == 3:
            x = board_coordinates[6].x
            y = board_coordinates[6].y
            pos_update = 6
        elif req.c == 4:
            x = board_coordinates[7].x
            y = board_coordinates[7].y
            pos_update = 7
    elif req.r == 3:
        if req.c == 1:
            x = board_coordinates[8].x
            y = board_coordinates[8].y
            pos_update = 8
        elif req.c == 2:
            x = board_coordinates[9].x
            y = board_coordinates[9].y
            pos_update = 9
        elif req.c == 3:
            x = board_coordinates[10].x
            y = board_coordinates[10].y
            pos_update = 10
        elif req.c == 4:
            x = board_coordinates[11].x
            y = board_coordinates[11].y
            pos_update = 11
    return PosBoardResponse(x, y, pos_update)

def boardCallback(msg):
    global board_coordinates
    board_coordinates = msg.coord

def moveInGazebo( x, y, z, color):
    #calls gazebo service to place red pieces
    #receives position to be placed
    global green_model_name, yellow_model_name, red_model_name
    state_msg = ModelState()
    if color=="green":
        state_msg.model_name = green_model_name.pop(0)
    elif color=="yellow":
        state_msg.model_name = yellow_model_name.pop(0)
    elif color=="red":
        state_msg.model_name = red_model_name.pop(0)
    state_msg.reference_frame = 'world'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    #q = quaternion_from_euler(0, -pi, 0)
    #state_msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        assert resp.success is True
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def moveInGazeboH( x, y, z, color):
    #calls gazebo service to place red pieces
    #receives position to be placed
    global green_model_name_h, yellow_model_name_h, red_model_name_h
    state_msg = ModelState()
    if color=="green":
        state_msg.model_name = green_model_name_h.pop(0)
    elif color=="yellow":
        state_msg.model_name = yellow_model_name_h.pop(0)
    elif color=="red":
        state_msg.model_name = red_model_name_h.pop(0)
    state_msg.reference_frame = 'world'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    #q = quaternion_from_euler(0, -pi, 0)
    #state_msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        assert resp.success is True
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def transformPos( pos):
    #transfroms position of game state array into row collumn in board
    r, c = 0, 0
    if pos == 0: r, c = 1, 1
    elif pos == 1: r, c = 1, 2
    elif pos == 2: r, c = 1, 3
    elif pos == 3: r, c = 1, 4
    elif pos == 4: r, c = 2, 1
    elif pos == 5: r, c = 2, 2
    elif pos == 6: r, c = 2, 3
    elif pos == 7: r, c = 2, 4
    elif pos == 8: r, c = 3, 1
    elif pos == 9: r, c = 3, 2
    elif pos == 10: r, c = 3, 3
    elif pos == 11: r, c = 3, 4

    return r, c

def gameStateUpdate(p_update, color):
    #updates array of plays
    global game_state
    game_state[p_update] = color

def endGame():
    #verifies state of the game
    #Four states: win red, win blue, tie, nothing(as in not over yet)
    global game_state
    for position in win:
        a, b, c = position
        if game_state[a] == game_state[b] and game_state[b] == game_state[c] and game_state[a] != "":
            if last_move == "HUMAN":
                return "WIN HUMAN"
            else:
                return "WIN ROBOT"

    res = all(board == "red" for board in game_state)
    if res == True:
        return "TIE"

    return "NOTHING"

#minimax algorithm
def maximization( depth, alpha, beta):
    #Robot play is maximized
    max_value = -2
    position = 11
    end_game = endGame()
    global game_state, last_move

    if end_game == "WIN HUMAN":
        return -1, 0, 0
    elif end_game == "WIN ROBOT":
        return 1, 0, 0
    elif end_game == "TIE":
        return 0, 0, 0
    elif depth == 0:
        return -1, 0, 0

    for pos in range(12):
        if game_state[pos] == "":
            game_state[pos] = "green"
            last_move = "ROBOT"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = ""
            last_move = "HUMAN"

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value

        elif game_state[pos] == "green":
            game_state[pos] = "yellow"
            last_move = "ROBOT"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = "green"
            last_move = "HUMAN"

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value

        elif game_state[pos] == "yellow":
            game_state[pos] = "red"
            last_move = "ROBOT"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = "yellow"
            last_move = "HUMAN"

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value


    r, c = transformPos(position)
    return max_value, r, c

def minimization( depth, alpha, beta):
    #Human play is minimized
    min_value = 2
    position = 11
    end_game = endGame()
    global game_state, last_move

    if end_game == "WIN HUMAN":
        return -1, 0, 0
    elif end_game == "WIN ROBOT":
        return 1, 0, 0
    elif end_game == "TIE":
        return 0, 0, 0

    for pos in range(12):
        if game_state[pos] == "":
            game_state[pos] = "green"
            last_move = "HUMAN"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = ""
            last_move = "ROBOT"

            if min_value <= alpha:
                r, c = transformPos(position)
                return min_value, r, c
            if min_value < alpha:
                beta = min_value

        elif game_state[pos] == "green":
            game_state[pos] = "yellow"
            last_move = "HUMAN"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = "green"
            last_move = "ROBOT"

            if min_value <= alpha:
                r, c = transformPos(position)
                return min_value, r, c
            if min_value < alpha:
                beta = min_value

        elif game_state[pos] == "yellow":
            game_state[pos] = "red"
            last_move = "HUMAN"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = "yellow"
            last_move = "ROBOT"

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
        elif c - 1 == 3:
            return b4
    elif r - 1 == 1:
        if c - 1 == 0:
            return b5
        elif c - 1 == 1:
            return b6
        elif c - 1 == 2:
            return b7
        elif c - 1 == 3:
            return b8
    elif r - 1 == 2:
        if c - 1 == 0:
            return b9
        elif c - 1 == 1:
            return b10
        elif c - 1 == 2:
            return b11
        elif c - 1 == 3:
            return b12

def getPos( r, c):
    if r - 1 == 0:
        if c - 1 == 0:
            return 0
        elif c - 1 == 1:
            return 1
        elif c - 1 == 2:
            return 2
        elif c - 1 == 3:
            return 3
    elif r - 1 == 1:
        if c - 1 == 0:
            return 4
        elif c - 1 == 1:
            return 5
        elif c - 1 == 2:
            return 6
        elif c - 1 == 3:
            return 7
    elif r - 1 == 2:
        if c - 1 == 0:
            return 8
        elif c - 1 == 1:
            return 9
        elif c - 1 == 2:
            return 10
        elif c - 1 == 3:
            return 11

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
    b10.config(state=DISABLED)
    b11.config(state=DISABLED)
    b12.config(state=DISABLED)

# Button function
def b_click( b, r, c):
    global last_move, game_state
    print "button clicked"
    if endGame() == "NOTHING":
        print "game not over"
        if b["text"] == " ":
            last_move = "HUMAN" #human played
            b["text"] = "green"
            b.configure(bg="green")
            #pos = getPos(r,c)
            resp = getPosBoard(r, c) # visualize in gazebo
            rx = resp.x
            ry = resp.y
            update_board = resp.pos_update
            moveInGazeboH(rx, ry, 0.05, "green")
            gameStateUpdate(update_board, "green")
            main.update()
            print "human played green"
            if endGame() == "NOTHING":
                print "robot is planning move"
                start_time = time.time()
                max_value, r, c = maximization(level, -2, 2)
                print "Robot took ", time.time()-start_time, "s to plan"
                last_move = "ROBOT" #robot played
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    b_b.configure(bg="green")
                    #get initial position of the piece
                    q = poses_green.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_green.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.05,"green")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.05)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "green")
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    b_b.configure(bg="yellow")
                    q = poses_yellow.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_yellow.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y, 0.10, "yellow")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.10)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    b_b.configure(bg="red")
                    q = poses_red.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_red.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.15,"red")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.15)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "red")

        elif b["text"] == "green":
            last_move = "HUMAN" #human played
            b["text"] = "yellow"
            b.configure(bg="yellow")
            resp = getPosBoard(r, c) # visualize in gazebo
            rx = resp.x
            ry = resp.y
            update_board = resp.pos_update
            moveInGazeboH(rx, ry, 0.10, "yellow")
            gameStateUpdate(update_board, "yellow")
            main.update()
            if endGame() == "NOTHING":
                print "robot is planning move"
                start_time = time.time()
                max_value, r, c = maximization(level, -2, 2)
                print "Robot took ", time.time()-start_time, "s to plan"
                last_move = "ROBOT" #robot played
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    b_b.configure(bg="green")
                    #get initial position of the piece
                    q = poses_green.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_green.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.05,"green")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.05)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "green")
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    b_b.configure(bg="yellow")
                    q = poses_yellow.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_yellow.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y, 0.10, "yellow")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.10)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    b_b.configure(bg="red")
                    q = poses_red.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_red.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.15,"red")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.15)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "red")

        elif b["text"] == "yellow":
            last_move = "HUMAN" #human played
            b["text"] = "red"
            b.configure(bg="red")
            resp = getPosBoard(r, c) # visualize in gazebo
            rx = resp.x
            ry = resp.y
            update_board = resp.pos_update
            moveInGazeboH(rx, ry, 0.15, "red")
            gameStateUpdate(update_board, "red")
            main.update()
            if endGame() == "NOTHING":
                print "robot is planning move"
                start_time = time.time()
                max_value, r, c = maximization(level, -2, 2)
                print "Robot took ", time.time()-start_time, "s to plan"
                last_move = "ROBOT" #robot played
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    b_b.configure(bg="green")
                    #get initial position of the piece
                    q = poses_green.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_green.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.05,"green")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.05)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "green")
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    b_b.configure(bg="yellow")
                    q = poses_yellow.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_yellow.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y, 0.10, "yellow")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.10)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    b_b.configure(bg="red")
                    q = poses_red.data
                    x = str(q[0].data[0])
                    y = str(q[0].data[1])
                    poses_red.data.pop(0)
                    resp = getPosBoard(r, c) # visualize in gazebo
                    update_board = resp.pos_update
                    rx = str(resp.x)
                    ry = str(resp.y)
                    #execute pick and place
                    moveInGazebo(resp.x, resp.y,0.15,"red")
                    '''
                    cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + str(0.05) + ' ' + rx + ' ' + ry + ' ' + str(0.15)
                    os.system(cmd)
                    '''
                    gameStateUpdate(update_board, "red")


    elif b["text"] == "red" and endGame() == "NOTHING":
        messagebox.showerror("Traffic Lights", "Position already occupied")

    if endGame() != "NOTHING":
        if endGame() == "WIN HUMAN":
            messagebox.showinfo("Traffic Lights", "HUMAN wins")
        elif endGame() == "WIN ROBOT":
            messagebox.showinfo("Traffic Lights", "ROBOT wins")
        else:
            messagebox.showinfo("Traffic Lights", "It's a TIE")

        game_state = ["","","","","","","","","","","",""]
        last_move = "ROBOT"
        global board_coordinates, green_model_name, green_model_name_h, yellow_model_name, yellow_model_name_h, red_model_name, red_model_name_h
        green_model_name = ['green_cylinder_short','green_cylinder_short_clone', 'green_cylinder_short_clone_0', 'green_cylinder_short_clone_1', 'green_cylinder_short_clone_2']
        yellow_model_name = ['yellow_cylinder_short','yellow_cylinder_short_clone', 'yellow_cylinder_short_clone_0', 'yellow_cylinder_short_clone_1', 'yellow_cylinder_short_clone_2']
        red_model_name = ['red_cylinder_short','red_cylinder_short_clone', 'red_cylinder_short_clone_0', 'red_cylinder_short_clone_1', 'red_cylinder_short_clone_2']
        green_model_name_h = ['green_cylinder_short_clone_3','green_cylinder_short_clone_4', 'green_cylinder_short_clone_5', 'green_cylinder_short_clone_6', 'green_cylinder_short_clone_7', 'green_cylinder_short_clone_7']
        yellow_model_name_h = ['yellow_cylinder_short_clone_3','yellow_cylinder_short_clone_4', 'yellow_cylinder_short_clone_5', 'yellow_cylinder_short_clone_6', 'yellow_cylinder_short_clone_7']
        red_model_name_h = ['red_cylinder_short_clone_3','red_cylinder_short_clone_4', 'red_cylinder_short_clone_5', 'red_cylinder_short_clone_6', 'red_cylinder_short_clone_7']

        disable_all_buttons()
        #reset simulation
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


        board_coordinates = rospy.wait_for_message('/processing_node_traffic/board_positions', InfoBoard)
        board_coordinates = board_coordinates.coord
        #print len(board_coordinates)


#Start new Game for the gui
def new():
    global b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12
    global level

    #ask for dificulty wanted
    level = simpledialog.askstring("Input", "Choose dificulty between 1(easiest) and 10(hardest):",parent=main)
    level = int(level)
    if level%2 != 0: level = level + 1

    # Buttons for the grid

    b1 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b1,1,1))
    b2 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b2,1,2))
    b3 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b3,1,3))
    b4 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b4,1,4))

    b5 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b5,2,1))
    b6 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b6,2,2))
    b7 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b7,2,3))
    b8 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b8,2,4))

    b9 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b9,3,1))
    b10 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b10,3,2))
    b11 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b11,3,3))
    b12 = Button(main, text=" ", font=("Helvetica",20), height=3, width=6, bg="Gray", command=lambda: b_click(b12,3,4))

    # Button in a grid

    b1.grid(row=0, column=0)
    b2.grid(row=0, column=1)
    b3.grid(row=0, column=2)
    b4.grid(row=0, column=3)

    b5.grid(row=1, column=0)
    b6.grid(row=1, column=1)
    b7.grid(row=1, column=2)
    b8.grid(row=1, column=3)

    b9.grid(row=2, column=0)
    b10.grid(row=2, column=1)
    b11.grid(row=2, column=2)
    b12.grid(row=2, column=3)

if __name__ == '__main__':
  try:
    #--------------------------------------------------------------------------
    rospy.init_node('traffic_lights')
    initServices()
    getPosBoard = rospy.ServiceProxy('/experiment_1_msgs/PosBoard', experiment_1_msgs.srv.PosBoard )
    getPieces = rospy.ServiceProxy('/experiment_1_msgs/PosPiece', experiment_1_msgs.srv.PosPiece)

    #Initialization------------------------------------------------------------
    #Define initial poses of pieces to be picked up by robot
    #reset simulation
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    global pieces_coordinates, board_coordinates
    board_coordinates = rospy.wait_for_message('/processing_node_traffic/board_positions', InfoBoard)
    board_coordinates = board_coordinates.coord
    pieces_g_coordinates = rospy.wait_for_message('/processing_node_traffic/pieces_positions_green', InfoPieces)
    poses_green = pieces_g_coordinates.poses
    pieces_y_coordinates = rospy.wait_for_message('/processing_node_traffic/pieces_positions_yellow', InfoPieces)
    poses_yellow = pieces_y_coordinates.poses
    pieces_r_coordinates = rospy.wait_for_message('/processing_node_traffic/pieces_positions_red', InfoPieces)
    poses_red = pieces_r_coordinates.poses

    #Gameplay-----------------------------------------------------------------

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
