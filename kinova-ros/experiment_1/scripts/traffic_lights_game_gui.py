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

game_state = ["","","","","","","","","","","",""]
win = [(0,1,2),(1,2,3),(4,5,6),(5,6,7),(8,9,10),(9,10,11),(0,4,8),(1,5,9),(2,6,10),(3,7,11),(0,5,10),(1,6,11),(3,6,9),(2,5,8)]

last_move = "ROBOT"
main = tk.Tk()
main.title('Traffic Lights')


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

    res = all(board != "" for board in game_state)
    if res == True:
        return "TIE"

    return "NOTHING"

#minimax algorithm
def maximization( depth, alpha, beta):
    #Robot play is maximized
    print "robot is maximizing"
    max_value = -2
    position = 12
    end_game = endGame()
    global game_state

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
        elif game_state[pos] == "green":
            game_state[pos] = "yellow"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = "green"

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value

        elif game_state[pos] == "yellow":
            game_state[pos] = "red"
            min_value, min_r, min_c = minimization(depth-1, alpha, beta)
            if min_value > max_value:
                max_value = min_value
                position = pos
            game_state[pos] = "yellow"

            if max_value >= beta:
                r, c = transformPos(position)
                return max_value, r, c
            if max_value > alpha:
                alpha = max_value


    r, c = transformPos(position)
    return max_value, r, c

def minimization( depth, alpha, beta):
    #Human play is minimized
    print "robot is minimizing"
    min_value = 2
    position = 12
    end_game = endGame()
    global game_state

    if end_game == "WIN HUMAN":
        return -1, 0, 0
    elif end_game == "WIN ROBOT":
        return 1, 0, 0
    elif end_game == "TIE":
        return 0, 0, 0

    for pos in range(12):
        if game_state[pos] == "":
            game_state[pos] = "green"
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
        elif game_state[pos] == "green":
            game_state[pos] = "yellow"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = "green"

            if min_value <= alpha:
                r, c = transformPos(position)
                return min_value, r, c
            if min_value < alpha:
                beta = min_value

        elif game_state[pos] == "yellow":
            game_state[pos] = "red"
            max_value, max_r, max_c = maximization(depth-1, alpha, beta)
            if max_value < min_value:
                min_value = max_value
                position = pos
            game_state[pos] = "yellow"

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
            pos = getPos(r,c)
            gameStateUpdate(pos, "green")
            main.update()
            print "human played green"
            if endGame() == "NOTHING":
                print "robot is planning move"
                last_move = "ROBOT" #robot played
                max_value, r, c = maximization(level, -2, 2)
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "green")
                    print "robot played green"
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "red")

        elif b["text"] == "green":
            last_move = "HUMAN" #human played
            b["text"] = "yellow"
            pos = getPos(r,c)
            gameStateUpdate(pos, "yellow")
            main.update()
            if endGame() == "NOTHING":
                last_move = "ROBOT" #robot played
                max_value, r, c = maximization(level, -2, 2)
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "green")
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "red")

        elif b["text"] == "yellow":
            last_move = "HUMAN" #human played
            b["text"] = "red"
            pos = getPos(r,c)
            gameStateUpdate(pos, "red")
            main.update()
            if endGame() == "NOTHING":
                last_move = "ROBOT" #robot played
                max_value, r, c = maximization(level, -2, 2)
                b_b = get_button(r, c) #returns string with button
                if b_b["text"] == " ":
                    b_b["text"] = "green"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "green")
                elif b_b["text"] == "green":
                    b_b["text"] = "yellow"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "yellow")
                elif b_b["text"] == "yellow":
                    b_b["text"] = "red"
                    pos = getPos(r,c)
                    gameStateUpdate(pos, "red")


    #elif b["text"] != " " and endGame() == "NOTHING":
    #    messagebox.showerror("Traffic Lights", "Position already occupied")

    if endGame() != "NOTHING":
        if endGame() == "WIN HUMAN":
            messagebox.showinfo("Traffic Lights", "HUMAN wins")
        elif endGame() == "WIN ROBOT":
            messagebox.showinfo("Traffic Lights", "ROBOT wins")
        else:
            messagebox.showinfo("Traffic Lights", "It's a TIE")

        game_state = ["","","","","","","","","","","",""]
        last_move = "ROBOT"
        disable_all_buttons()



#Start new Game for the gui
def new():
    global b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12
    global level

    #ask for dificulty wanted
    level = simpledialog.askstring("Input", "Choose dificulty between 1(easiest) and 8(hardest):",parent=main)
    level = int(level)

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

    #Initialization------------------------------------------------------------

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
