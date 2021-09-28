import pygame
import math
from queue import PriorityQueue

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


class Spot:
    def __init__(self, row, col, width, total_rows):
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


    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_obstacle(self):
        self.color = BLACK

    def make_end(self):
        self.color = BLUE
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
    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2)+abs(y1-y2)


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
                    neighbor.make_open()        #if node was NOT optimal before but was neighbor of other optimal node 
        draw()
        if current != start:
            pass
            current.make_closed()   #if node was optimal before

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
    ROWS=10
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
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    algorithm(lambda:draw(win,grid,ROWS,width),grid,start,end)
                if event.key==pygame.K_c : 
                    start=None
                    end=None
                    grid=make_grid(ROWS,width)

    pygame.quit()     

main(WIN,WIDTH)   
    