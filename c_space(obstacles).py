import pygame
import math
from queue import PriorityQueue

import numpy as np

a1 = 4.5  # link 1
a2 = 7.0  # link 2
a3 = 4.6  # link 3
a4 = 4.5  # link 4
a5 = 7.1  # link 5

d3 = 2.0  # displacement 3



WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("path finding algoriithm")


RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
BLUE = (0, 0, 255)

cart_obs=[]
cart_start=None
cart_end=None

class Spot:
    def __init__(self, row, col, width, total_rows):
        if total_rows==24:
            self.row = row-12
        else:
            self.row = row
        self.col = col
        self.x = row*width
        self.y = col*width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_obstacle(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == BLUE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE
        cart_start=self


    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_obstacle(self):
        self.color = BLACK
        cart_obs.append(self)

    def make_end(self):
        self.color = BLUE
        cart_end=self

    def make_path(self):
        self.color= PURPLE

    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self,grid):
        self.neighbors=[]
        if self.row<self.total_rows-1 and not grid[self.row+1][self.col].is_obstacle():#bottom neighbor
            self.neighbors.append(grid[self.row+1][self.col])
        
        if self.row>0 and not grid[self.row-1][self.col].is_obstacle():#top neighbor
            self.neighbors.append(grid[self.row-1][self.col])
        
        if self.col>0 and not grid[self.row][self.col-1].is_obstacle():#left neighbor
            self.neighbors.append(grid[self.row][self.col-1])
        
        if self.col<self.total_rows-1 and not grid[self.row][self.col+1].is_obstacle():#right neighbor
            self.neighbors.append(grid[self.row][self.col+1])   

        #c-space collab------ 

        if self.row==0 and not grid[self.total_rows-1][self.col].is_obstacle():#top-bottom neighbor
            self.neighbors.append(grid[self.total_rows-1][self.col])

        if self.row==self.total_rows-1 and not grid[0][self.col].is_obstacle():#bottom-top neighbor
            self.neighbors.append(grid[0][self.col])

        if self.col==0 and not grid[self.row][self.total_rows-1].is_obstacle():#left-right neighbor
            self.neighbors.append(grid[self.row][self.total_rows-1])

        if self.col==self.total_rows-1 and not grid[self.row][0].is_obstacle():#left neighbor
            self.neighbors.append(grid[self.row][0])


    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2)+abs(y1-y2)

def inv_kine(config_grid,cart_spot):

    x0_3 = float(cart_spot.row)
    y0_3 = float(cart_spot.col)
    print(x0_3, y0_3)
    inRoot = (x0_3*x0_3) + (y0_3*y0_3)
    r1 = np.sqrt(inRoot)
    print("in fi1",(np.square(a2) + np.square(r1)-np.square(a4))/(2*a2*r1))
    fi1 = np.arccos((np.square(a2) + np.square(r1)-np.square(a4))/(2*a2*r1))
    fi2 = np.arccos((np.square(a2) + np.square(a4)-np.square(r1))/(2*a2*a4))
    fi3 = np.arctan(y0_3/x0_3)
    if(x0_3 >= 0):
        T1 = (fi1+fi3)*180.0/np.pi
        T2 = (fi2*180.0/np.pi)-180
    elif (x0_3 < 0):
        T1 = 180+((fi3-fi1)*180.0/np.pi)
        T2 = 180-(fi2*180.0/np.pi)
    print(T1,T2)
    return config_grid[int(T1)][int(T2)+90]

def forward_kine(theta1,theta2):
    # 3D SCARA
    #print("angles", theta1,theta2)
    pt = [
        [theta1*(np.pi/180), 0, a2, a1],
        [theta2*(np.pi/180), np.pi, a4, a3],
        [0, 0, 0, a5+d3]
    ]
    H = []
    for row in pt:
        Hi = [
            [np.cos(row[0]), -np.sin(row[0])*np.cos(row[1]),
             np.sin(row[0])*np.sin(row[1]), row[2]*np.cos(row[0])],
            [np.sin(row[0]),  np.cos(row[0])*np.cos(row[1]), -
             np.cos(row[0])*np.sin(row[1]), row[2]*np.sin(row[0])],
            [0.0, np.sin(row[1]), np.cos(row[1]), row[3]],
            [0.0, 0.0, 0.0, 1.0]
        ]
        H.append(Hi)
    H0_2 = np.dot(H[0], H[1])
    H0_3 = np.dot(H0_2, H[2])
    #print("forward ",int(H0_3[0][3]),int(H0_3[1][3]))
    return int(H0_3[0][3]),int(H0_3[1][3])


def collision_check(config_spot):
    cart_spot=forward_kine(config_spot.row,config_spot.col-90)
    for obs in cart_obs:
        if obs.row==cart_spot[0] and obs.col==cart_spot[1]:
            print("le obsssss ",obs.row,obs.col)
            return True
    return False


def cart2config(config_grid,cart_obs):
    for row in config_grid:
        for config_spot in row:
            if collision_check(config_spot) :
                config_spot.make_obstacle()



def construct_shortest_path(came_from,current,draw):
    while current in came_from:
        current=came_from[current]
        current.make_path()
        draw()



################################### A   S T A R   Algorithm ##################################
def algorithm(draw, grid,start,end):
    count=0
    open_set=PriorityQueue()
    open_set.put((0,count,start))   # Begin by putting the start node in the open_set 
    came_from={}                    # Keeps track of all paths(including the optimal one)
    g_score={}
    f_score={}
    for row in grid:                #Making g_score of ALL the nodes infinity
        for spot in row:
            g_score.update({spot:float("inf")})
    g_score[start]=0                #But g_score of the end node is "0"
    
    for row in grid:                # Making f_score of ALL the nodes infinity
        for spot in row:
            f_score.update({spot:float("inf")})
    f_score[start]=h(start.get_pos(),end.get_pos()) #But f_score of the end node is "heuristic distance" only
    
    open_set_hash={start}           # keeping track of that priorityqueue
    
    while not open_set.empty():
        for event in pygame.event.get(): 
            if event.type==pygame.QUIT:
                pygame.quit()

        current=open_set.get()[2]    # Getting the current working node from the open_set to compare with neighbors  
        open_set_hash.remove(current) # Removing the current working node from the open_set_hash
        if current == end:           # We have reached to the "goal position"
            construct_shortest_path(came_from,end,draw)
            end.make_end()
            start.make_start()

            return True
        for neighbor in current.neighbors:  # working with ALL the neighbors to get the next optimal node
            temp_g_score=g_score[current]+1 # if we go to the next immediate neighbor the g_score increases by 1

            if temp_g_score<g_score[neighbor]:  #if new temp_g_score is lesser than the previous g_score then update the g_score with the lesser one
                came_from[neighbor]=current     # Saving the optimal node from where we came at the first place
                g_score[neighbor]=temp_g_score  # updating the g_score with the lesser one
                f_score[neighbor]=temp_g_score+h(neighbor.get_pos(),end.get_pos())
                if neighbor not in open_set_hash: # Updating open_set with the optimal neighbor which was not optimal before
                    count +=1
                    open_set.put((f_score[neighbor],count,neighbor))
                    open_set_hash.add(neighbor)     # Updating open_set_hash with the optimal neighbor
                    #neighbor.make_open()        #if node was NOT optimal before but was neighbor of other optimal node 
        draw()
        if current != start:
            pass
            #current.make_closed()   #if node was optimal before

    return False






def make_grid(rows, width):
    grid = []
    spot_width = width // rows
    for i in range(rows):
        grid.append([])  # making all the rows
        for j in range(rows):
            spot = Spot(i, j, spot_width, rows)
            grid[i].append(spot)  # making all the collumns
    return grid


def draw_gridline(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i*gap), (width, i*gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j*gap, 0), (j*gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)
    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_gridline(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width//rows
    x, y = pos
    row = x//gap
    col = y//gap
    return row, col


def main(win, width):
    ROWS=24
    

    grid=make_grid(ROWS,width)
    
    start=None
    end=None
    
    run=True
    started=False
    while run:
        draw(win,grid,ROWS,width)


        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                run=False
            if started:
                continue
            if pygame.mouse.get_pressed()[0]:
                pos=pygame.mouse.get_pos()
                row,col=get_clicked_pos(pos,ROWS,width)
                spot=grid[row][col]
                if not start and spot !=end:
                    start=spot
                    start.make_start()
                elif not end and spot !=start:
                    end=spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_obstacle()
            elif pygame.mouse.get_pressed()[1]:
                pass
            elif pygame.mouse.get_pressed()[2]:
                pos=pygame.mouse.get_pos()
                row,col=get_clicked_pos(pos,ROWS,width)
                spot=grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot ==end:
                    end = None
            if event.type==pygame.KEYDOWN:
                if event.key==pygame.K_p :
                    for obs in cart_obs:
                        print(obs.row,obs.col)

                    config_grid=make_grid(180,width)
                    config_start=inv_kine(config_grid,start)
                    print (config_start.row,config_start.col)
                    config_start.make_start()
                    config_end=inv_kine(config_grid,end)
                    print (config_end.row,config_end.col)
                    config_end.make_end()
                    cart2config(config_grid,cart_obs)
                        
                if event.key == pygame.K_SPACE and start and end:
                    for row in config_grid:
                        for spot in row:
                            spot.update_neighbors(config_grid)
                    algorithm(lambda:draw(win,config_grid,180,width),config_grid,config_start,config_end)
                if event.key==pygame.K_c : 
                    start=None
                    end=None
                    grid=make_grid(ROWS,width)


    pygame.quit()     

main(WIN,WIDTH)   
    