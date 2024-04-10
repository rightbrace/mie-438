import glob
import serial

import pygame, sys
from pygame.locals import *


connection = serial.Serial(
    port = glob.glob('/dev/tty.*')[0], # Mac-only - windows would need COM
    baudrate = 38400,
    bytesize = 8,
    parity = "N",
    stopbits = 1,
    timeout=0.001)

grid_color = 120, 120, 120
background_color = 45, 45, 45

unselected_color = 102, 102, 255
hover_color = 153, 204, 255
click_color = 255, 153, 153

text_color = 255, 255, 255

color_table = [
    (202, 44, 56),
    # (57, 184, 67),
    (83, 129, 182),
    (191, 188, 68)]

image_width = 8
image_height = 8

canvas_scale = 50
window_width = canvas_scale * image_width
window_height = canvas_scale * (image_height + 2)


mouse_x = 0
mouse_y = 9
mouse_click = False
mouse_down = False

def button(x, y, width, height, text="", color=unselected_color, hover_color=hover_color, click_color=click_color):
    if x <= mouse_x <= x + width and y <= mouse_y <= y + height:
        if mouse_down:
            pygame.draw.rect(window, click_color, (x, y, width, height))
        else:
            pygame.draw.rect(window, hover_color, (x, y, width, height))

        draw_text(window, text_color, text, x + width/2, y+height/2, center_x=True, center_y=True)
        return mouse_click
    else:
        pygame.draw.rect(window, color, (x, y, width, height))
        draw_text(window, text_color, text, x + width/2, y+height/2, center_x=True, center_y=True)

        return False
    
def draw_text(surface, color, string, x, y, center_x=False, center_y=False):
    if string == "": 
        return
    text = display_font.render(string, True, color)
    cx, cy = x, y
    if center_x:
        cx -= text.get_width() / 2
    if center_y:
        cy -= text.get_height() / 2
    surface.blit(text, (cx, cy))

canvas = [[0 for _ in range(image_height)] for _ in range(image_width)]

pygame.init()
pygame.font.init()
display_font = pygame.font.SysFont("Monaco", 14)
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("MIE438 Candy Sorter")
clock = pygame.time.Clock()

while True: # main game loop

    mouse_click = False
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 1:
                mouse_click = True
            mouse_down = True
        if event.type == MOUSEBUTTONUP:
            mouse_down = False
    
    # Check mouse state
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Clear display
    pygame.draw.rect(window, background_color, (0, 0, window_width, window_height))

    # Render grid
    for row in range(image_height+1):
        pygame.draw.line(window, grid_color, (0, row * canvas_scale), (window_width, row * canvas_scale))
    
    for col in range(1, image_width):
        pygame.draw.line(window, grid_color, (col * canvas_scale, 0), (col * canvas_scale, image_height * canvas_scale))

    for x in range(image_width):
        for y in range(image_height):
            canvas[x][y] += 1 if button(x * canvas_scale + 1, 
                                        y * canvas_scale + 1, 
                                        canvas_scale - 2, 
                                        canvas_scale - 2, 
                                        color=color_table[canvas[x][y]],
                                        click_color=color_table[(canvas[x][y])],
                                        hover_color=color_table[canvas[x][y]]) else 0
            canvas[x][y] %= len(color_table)

    if button(window_width-110, image_height * canvas_scale + 10, 
              100, 25, "Send"):
        # Make byte array from image
        # Each pixel can be two bits
        # 00 - red
        # 01 - green
        # 10 - blue
        # 11 - yellow
        # Send an 8x8 image - 16 bytes + one start byte + one stop byte
        #  01  10  10  01                     (ascii 'i')
        # [xx][xx][xx][xx] [xx][xx][xx][xx]   (top to bottom, left to right)
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        # [xx][xx][xx][xx] [xx][xx][xx][xx]
        #  00  11  10  11                     (ascii ';')
        msg = [ord('i')]
        for y in range(image_height):
            for c in range(2):
                byte = 0
                for x in range(4):
                    px = canvas[c*4+x][y]
                    byte |= px << (6-x*2)
                msg.append(byte)
        msg.append(ord(';'))
        connection.write(bytes(msg))
        print("<<Sending Image Data>>")

    if button(window_width-110, image_height * canvas_scale + 40, 
            100, 25, "Start/Stop"):
        connection.write(b"p;")
        print("<<START / STOP>>")

        
    pygame.display.update()

    # Render serial data
    while True:
        byte = connection.read(1)
        if len(byte) == 0:
            break
        print(byte.decode("utf8"), end="", flush=True)
        
    clock.tick(60)