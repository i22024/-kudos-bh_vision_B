import numpy as np
desire_pan = 0
desire_tilt = 0
max_pan = 1.57
min_pan = -1.57
add_for_pan = 0.01

def draw_head_path():
    global desire_pan
    global desire_tilt
    global max_pan
    global min_pan
    global add_for_pan
    desire_pan = desire_pan + add_for_pan
    if desire_pan>max_pan and add_for_pan>0:
        add_for_pan = -0.01
    elif desire_pan < min_pan and add_for_pan<0:
        add_for_pan = 0.01
        

def tracking_ball(message_form):
    global desire_pan
    global desire_tilt
    draw_head_path()
    return desire_pan, desire_tilt