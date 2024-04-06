import pygame, sys
import time

WIDTH = 10
HEIGHT = 10

board = [0]*WIDTH*HEIGHT
pygame.init()
surface = pygame.display.set_mode((560,560))
w,h = surface.get_size()

def draw_board(board, surface):
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if board[y * HEIGHT + x] == 1:
                color = (0,255,0)
            else:
                color = (0,0,0)
            pygame.draw.rect(surface,color, pygame.Rect(x*w/WIDTH, y*h/HEIGHT, w/WIDTH, h/HEIGHT), border_radius=2)
    pygame.display.flip()
    return 

surface.fill((200,200,200))
pygame.display.flip()

draw_board(board, surface)
while True:
    for evnt in pygame.event.get():
        if evnt.type == pygame.MOUSEBUTTONDOWN:
            posx, posy = pygame.mouse.get_pos()
            boardx = posx * WIDTH // w
            boardy = posy * HEIGHT // h
            board[boardy * HEIGHT + boardx] = 1 - board[boardy * HEIGHT + boardx]
            draw_board(board, surface)

        if evnt.type == pygame.KEYDOWN:
            with open("ST7735_Fonts.h", 'a') as f:
                str_board = [str(element) for element in board]
                f.write("{" + ",".join(str_board) + "}\n")
        
        if evnt.type == pygame.QUIT:
            pygame.quit()
            sys.exit()