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


global poses_red
poses_red = List_2()
global poses_blue
poses_blue = List_2()
#red_model_name = ['red_cylinder','red_cylinder_0', 'red_cylinder_1', 'red_cylinder_2', 'red_cylinder_3']#for jaco tic_tac_toe_kinect
#blue_model_name = ['blue_cylinder','blue_cylinder_0', 'blue_cylinder_1', 'blue_cylinder_2', 'blue_cylinder_3']#for jaco tic_tac_toe_kinect
red_model_name = ['red_cylinder_short','red_cylinder_short_0', 'red_cylinder_short_1', 'red_cylinder_short_2', 'red_cylinder_short_3']#for jaco tic_tac_toe_kinect_short
blue_model_name = ['blue_cylinder_short','blue_cylinder_short_0', 'blue_cylinder_short_1', 'blue_cylinder_short_2', 'blue_cylinder_short_3']#for jaco tic_tac_toe_kinect_short
game_state = ["","","","","","","","",""]
h = str(0.05) #height of the pieces needed to obtain the moves for the robot
win = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6)]
global r, c
global board_coordinates
global pieces_coordinates

main = tk.Tk()
main.title('Tic-Tac-Toe')


def initServices():
    #Instanciate all the services
    rospy.Service(name="experiment_1_msgs/PosBoard", service_class=PosBoard,
                                                    handler=pos_board_handler)
    rospy.Service(name="experiment_1_msgs/PosPiece", service_class=PosPiece,
                                                    handler=pos_piece_handler)


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
    elif req.r == 2:
        if req.c == 1:
            x = board_coordinates[3].x
            y = board_coordinates[3].y
            pos_update = 3
        elif req.c == 2:
            x = board_coordinates[4].x
            y = board_coordinates[4].y
            pos_update = 4
        elif req.c == 3:
            x = board_coordinates[5].x
            y = board_coordinates[5].y
            pos_update = 5
    elif req.r == 3:
        if req.c == 1:
            x = board_coordinates[6].x
            y = board_coordinates[6].y
            pos_update = 6
        elif req.c == 2:
            x = board_coordinates[7].x
            y = board_coordinates[7].y
            pos_update = 7
        elif req.c == 3:
            x = board_coordinates[8].x
            y = board_coordinates[8].y
            pos_update = 8
    return PosBoardResponse(x, y, pos_update)

def pos_piece_handler( req):
    #gets the initial position of the pieces relative to the world and stores them in vectors
    global poses_red, poses_blue

    #Red - at this moment this pieces are not being detected since we only have simulation
    pose = (0.4, -0.04)
    poses_red.data.append(List1(data = pose))
    pose = (0.4, -0.14)
    poses_red.data.append(List1(data = pose))
    pose = (0.4, -0.24)
    poses_red.data.append(List1(data = pose))
    pose = (0.4, -0.34)
    poses_red.data.append(List1(data = pose))
    pose = (0.4, -0.44)
    poses_red.data.append(List1(data = pose))

    #Blue
    '''
    pose = (-0.4, -0.04)
    poses_blue.data.append(List1(data = pose))
    pose = (-0.4, -0.14)
    poses_blue.data.append(List1(data = pose))
    pose = (-0.4, -0.24)
    poses_blue.data.append(List1(data = pose))
    pose = (-0.4, -0.34)
    poses_blue.data.append(List1(data = pose))
    pose = (-0.4, -0.44)
    poses_blue.data.append(List1(data = pose))
    '''
    #global pieces_coordinates
    poses_blue = pieces_coordinates.poses

    #print "service red: ", poses_red
    #print "service blue: ", poses_blue

    return PosPieceResponse(poses_red, poses_blue)

def boardCallback(msg):
    global board_coordinates
    board_coordinates = msg.coord
    #print board_coordinates

def piecesCallback(msg):
    global pieces_coordinates
    pieces_coordinates = msg.poses
    #print pieces_coordinates

def validateMove( r, c):
    resp =  getPosBoard(r,c)# visualize in gazebo
    rx = resp.x
    ry = resp.y
    p_update = resp.pos_update
    if p_update < 0 or p_update > 9:
        return False
    if game_state[p_update]=="":
        return True
    else:
        return False

def gameStateUpdate( p_update, color):
    #updates array of plays
    game_state[p_update] = color

def endGame():
    #verifies state of the game
    #Four states: win red, win blue, tie, nothing(as in not over yet)
    global game_state
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

def moveInGazebo( x, y, color):
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
    state_msg.pose.position.z = float(h) #default value related to the height of the piece
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
    elif pos == 3: r, c = 2, 1
    elif pos == 4: r, c = 2, 2
    elif pos == 5: r, c = 2, 3
    elif pos == 6: r, c = 3, 1
    elif pos == 7: r, c = 3, 2
    elif pos == 8: r, c = 3, 3
    return r, c

#minimax algorithm
def maximization( depth, alpha, beta):
    #Robot play is maximized
    #Robot plays blue
    max_value = -2
    position = 9
    end_game = endGame()
    global game_state

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
            '''
            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value
            '''
    r, c = transformPos(position)
    return max_value, r, c

def minimization( depth, alpha, beta):
    #Human play is minimized
    #Human plays red
    min_value = 2
    position = 9
    end_game = endGame()
    global game_state

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
            '''
            if min_value <= alpha:
                r, c = transformPos(position)
                return min_value, r, c
            if min_value < alpha:
                beta = min_value
            '''
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
def b_click( b, r, c):
    global game_state,red_model_name, blue_model_name
    if b["text"] == " " and endGame() == "NOTHING":
        b["text"] = "RED" #human played
        b.configure(bg="red")
        resp = getPosBoard(r, c) # visualize in gazebo
        rx = resp.x
        ry = resp.y
        update_board = resp.pos_update
        moveInGazebo(rx, ry, "red")
        gameStateUpdate(update_board, "red")
        #print "Poses_red : ", poses_red.data
        poses_red.data.pop(0)
        #print "Red move: ", resp
        main.update()

        #robot plays next if human did not won
        if endGame() == "NOTHING":
            #get initial position of the piece
            q = poses_blue.data
            #print "Position of the piece: ", q[0].data[0]
            x = str(q[0].data[0])
            y = str(q[0].data[1])
            poses_blue.data.pop(0)
            #print "X : ", x, "Y :", y

            #place vars
            start_time = time.time()
            max_value, r, c = maximization(level, -2, 2)
            print "Robot took ", time.time()-start_time, "s to plan"
            resp = getPosBoard(r, c) # visualize in gazebo
            rx = resp.x
            ry = resp.y
            update_board = resp.pos_update
            gameStateUpdate(update_board, "blue")
            #show where robot plays on grid
            b_b = get_button(r, c) #returns string with button
            b_b["text"] = "BLUE"
            b_b.configure(bg="blue")

            moveInGazebo(rx, ry, "blue") #move directly in gazeb without the arm
            #print "Blue move: ", resp
            '''
            rx = str(rx)
            ry = str(ry)
            #execute pick and place
            cmd = 'rosrun experiment_1 pick_up_and_place.py ' + x + ' ' + y + ' ' + h + ' ' + rx + ' ' + ry + ' ' + h
            os.system(cmd)
            '''

    elif b["text"] != " " and endGame() == "NOTHING":
        messagebox.showerror("Tic-Tac-Toe", "Position already occupied")

    if endGame() != "NOTHING":
        if endGame() == "WIN RED":
            messagebox.showinfo("Tic-Tac-Toe", "RED wins")
        elif endGame() == "WIN BLUE":
            messagebox.showinfo("Tic-Tac-Toe", "BLUE wins")
        else:
            messagebox.showinfo("Tic-Tac-Toe", "It's a TIE")

        #red_model_name = ['red_cylinder','red_cylinder_0', 'red_cylinder_1', 'red_cylinder_2', 'red_cylinder_3']#for jaco tic_tac_toe_kinect
        #blue_model_name = ['blue_cylinder','blue_cylinder_0', 'blue_cylinder_1', 'blue_cylinder_2', 'blue_cylinder_3']#for jaco tic_tac_toe_kinect
        red_model_name = ['red_cylinder_short','red_cylinder_short_0', 'red_cylinder_short_1', 'red_cylinder_short_2', 'red_cylinder_short_3']#for jaco tic_tac_toe_kinect_short
        blue_model_name = ['blue_cylinder_short','blue_cylinder_short_0', 'blue_cylinder_short_1', 'blue_cylinder_short_2', 'blue_cylinder_short_3']#for jaco tic_tac_toe_kinect_short
        game_state = ["","","","","","","","",""]
        disable_all_buttons()
        #reset simulation
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        global pieces_coordinates, board_coordinates
        board_coordinates = rospy.wait_for_message('/processing_node/board_positions', InfoBoard)
        board_coordinates = board_coordinates.coord
        pieces_coordinates = rospy.wait_for_message('/processing_node/pieces_positions', InfoPieces)
        getPieces()

#Start new Game for the gui
def new():
    global b1, b2, b3, b4, b5, b6, b7, b8, b9
    global level

    #ask for dificulty wanted
    level = simpledialog.askstring("Input", "Choose dificulty between 1(easiest) and 8(hardest):",parent=main)
    level = int(level)
    if level%2 != 0: level = level + 1

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
    rospy.init_node('tic_tac_toe')
    initServices()
    getPosBoard = rospy.ServiceProxy('/experiment_1_msgs/PosBoard', experiment_1_msgs.srv.PosBoard )
    getPieces = rospy.ServiceProxy('/experiment_1_msgs/PosPiece', experiment_1_msgs.srv.PosPiece)

    #Initialization------------------------------------------------------------
    #Define initial poses of pieces to be picked up by robot

    global pieces_coordinates, board_coordinates
    board_coordinates = rospy.wait_for_message('/processing_node/board_positions', InfoBoard)
    board_coordinates = board_coordinates.coord
    pieces_coordinates = rospy.wait_for_message('/processing_node/pieces_positions', InfoPieces)
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
