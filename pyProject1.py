#!/usr/bin/env python3
"""
micromouse_sim.py
Single-file Micro-Mouse / Flood-Fill simulation in Python + pygame.

Features:
- Generates a random perfect maze (recursive backtracker).
- Stores walls as bitmask per cell like Arduino code:
    bit 0 (1) = North wall
    bit 1 (2) = East wall
    bit 2 (4) = South wall
    bit 3 (8) = West wall
  (Matches convention used in earlier discussion with small coordinate flip in drawing)
- Flood-fill (BFS from center 2x2) to produce distance map.
- step_to_lowest() moves the mouse to a neighbor with lower flood value (tie prefers current direction).
- A* path find from current to center (for comparison/demo).
- Controls:
    SPACE = single step (compute flood then step once)
    G = run auto (animate until reach center)
    A = compute A* path and follow it (press again to stop)
    R = regenerate maze / reset mouse
    V = toggle display flood numbers
    ESC or window close to exit
"""

import pygame, sys, random, math
from collections import deque, namedtuple
from heapq import heappush, heappop

# ---------- Config ----------
N = 16              # maze size NxN (matches your Arduino 16x16 example)
CELL = 36           # pixels per cell
WINDOW = N * CELL
FPS = 30
# directions: 0=N, 1=E, 2=S, 3=W  (same as your Arduino)
dx = [0, 1, 0, -1]
dy = [1, 0, -1, 0]
# bitmask flags
W_NORTH = 1  # bit 0
W_EAST  = 2  # bit 1
W_SOUTH = 4  # bit 2
W_WEST  = 8  # bit 3

# ---------- State ----------
walls = [[0 for _ in range(N)] for _ in range(N)]   # bitmask
flood = [[999 for _ in range(N)] for _ in range(N)]
visited = [[0 for _ in range(N)] for _ in range(N)]

curr_x, curr_y = 0, 0
Direction = 0  # current heading
auto_run = False
show_nums = False
follow_astar = False
astar_path = []

# center cells like Arduino: (7,7),(7,8),(8,7),(8,8) for N=16
center_coords = [(N//2 - 1, N//2 - 1), (N//2 - 1, N//2),
                 (N//2, N//2 - 1), (N//2, N//2)]

# ---------- Maze generation ----------
def generate_maze():
    """
    Create a perfect maze using recursive backtracker.
    We'll carve passages between cells and then set wall bitmasks accordingly.
    """
    # adjacency - initially no connections
    conn = [[set() for _ in range(N)] for _ in range(N)]
    stack = []
    sx, sy = 0, 0
    visited_gen = [[False]*N for _ in range(N)]
    visited_gen[sx][sy] = True
    stack.append((sx, sy))
    while stack:
        x,y = stack[-1]
        neighbors = []
        for d in range(4):
            nx, ny = x + dx[d], y + dy[d]
            if 0 <= nx < N and 0 <= ny < N and not visited_gen[nx][ny]:
                neighbors.append((nx, ny, d))
        if neighbors:
            nx, ny, d = random.choice(neighbors)
            visited_gen[nx][ny] = True
            # connect x,y <-> nx,ny
            conn[x][y].add((nx,ny))
            conn[nx][ny].add((x,y))
            stack.append((nx,ny))
        else:
            stack.pop()
    # convert connections to walls bitmask: start with all walls then remove where connection exists
    for x in range(N):
        for y in range(N):
            w = 0
            # assume wall exists unless connection tells otherwise
            # check each direction
            for d in range(4):
                nx, ny = x + dx[d], y + dy[d]
                if 0 <= nx < N and 0 <= ny < N:
                    if (nx,ny) in conn[x][y]:
                        # no wall in direction d
                        pass
                    else:
                        # set wall
                        if d == 0: w |= W_NORTH
                        elif d == 1: w |= W_EAST
                        elif d == 2: w |= W_SOUTH
                        elif d == 3: w |= W_WEST
                else:
                    # border walls
                    if d == 0: w |= W_NORTH
                    elif d == 1: w |= W_EAST
                    elif d == 2: w |= W_SOUTH
                    elif d == 3: w |= W_WEST
            walls[x][y] = w

# ---------- Flood fill (BFS from center cells) ----------
def flood_fill():
    """Compute flood[][] as distance to nearest center cell (BFS)."""
    # reset
    for i in range(N):
        for j in range(N):
            visited[i][j] = 0
            flood[i][j] = 999
    q = deque()
    # seed centers as value 0
    for (cx,cy) in center_coords:
        flood[cx][cy] = 0
        visited[cx][cy] = 1
        q.append((cx,cy))
    while q:
        cx, cy = q.popleft()
        val = flood[cx][cy]
        for d in range(4):
            nx, ny = cx + dx[d], cy + dy[d]
            if 0 <= nx < N and 0 <= ny < N:
                # if no wall in direction d from (cx,cy)
                if not (walls[cx][cy] & (1 << d)):   # same test as Arduino hasWall
                    if not visited[nx][ny]:
                        flood[nx][ny] = val + 1
                        visited[nx][ny] = 1
                        q.append((nx,ny))

# ---------- Helpers ----------
def has_wall(x,y,dir):
    return (walls[x][y] & (1 << dir)) != 0

def turn_to_dir(newdir):
    """Simulate turning; in simulation we only update Direction variable.
       Returns tuple of (turns_string) mostly for debug/animation future use."""
    global Direction
    old = Direction
    Direction = newdir
    # could return sequence of actions like "L","R" etc. but not needed now
    return

def step_to_lowest():
    """
    Move to neighboring cell with lowest flood value (prefer current direction).
    Mirrors stepToLowestNeighborStatic logic from your Arduino code.
    """
    global curr_x, curr_y, Direction
    bestDir = -1
    bestVal = 999
    for d in range(4):
        nx = curr_x + dx[d]; ny = curr_y + dy[d]
        if 0 <= nx < N and 0 <= ny < N:
            if not has_wall(curr_x,curr_y,d):
                if flood[nx][ny] < bestVal:
                    bestVal = flood[nx][ny]
                    bestDir = d
    if bestDir == -1:
        return False  # stuck or surrounded
    # prefer current direction if equal
    for d in range(4):
        nx = curr_x + dx[d]; ny = curr_y + dy[d]
        if 0 <= nx < N and 0 <= ny < N:
            if not has_wall(curr_x,curr_y,d):
                if flood[nx][ny] == bestVal and d == Direction:
                    bestDir = d
                    break
    # turn and move
    turn_to_dir(bestDir)
    curr_x += dx[Direction]; curr_y += dy[Direction]
    return True

# ---------- A* for comparison ----------
def heuristic(a, b):
    (x1,y1) = a; (x2,y2) = b
    # Manhattan in grid
    return abs(x1-x2) + abs(y1-y2)
#-----------------------------------------------------
def astar(start, goal):
    """Return path list of (x,y) from start to goal or [] if none."""
    sx, sy = start
    gx, gy = goal
    open_set = []
    heappush(open_set, (0 + heuristic(start, goal), 0, start, None))
    came = {}
    gscore = {start: 0}
    while open_set:
        f, g, current, parent = heappop(open_set)
        if current in came:
            continue
        came[current] = parent
        if current == goal:
            # reconstruct
            path = []
            cur = current
            while cur:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path
        cx, cy = current
        for d in range(4):
            nx, ny = cx + dx[d], cy + dy[d]
            if 0 <= nx < N and 0 <= ny < N:
                if not has_wall(cx,cy,d):
                    neighbor = (nx,ny)
                    tentative = g + 1
                    if neighbor not in gscore or tentative < gscore[neighbor]:
                        gscore[neighbor] = tentative
                        heappush(open_set, (tentative + heuristic(neighbor, goal), tentative, neighbor, current))
    return []

# ---------- Drawing ----------
pygame.init()
screen = pygame.display.set_mode((WINDOW, WINDOW))
pygame.display.set_caption("Micro-Mouse Flood-Fill Simulation")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 18)

def cell_to_screen(x,y):
    # our flood/grid uses x to right, y upwards (Arduino style). 
    # Pygame Y grows down, so invert y when drawing: screen_y = (N-1-y)
    return x * CELL, (N-1-y) * CELL

def draw():
    screen.fill((230,230,230))
    # background based on flood value
    maxf = max(max((v for v in row if v<999), default=0) for row in flood)
    for x in range(N):
        for y in range(N):
            fx = flood[x][y]
            sx, sy = cell_to_screen(x,y)
            # color gradient: closer to center -> darker blue
            if fx >= 999:
                color = (240,240,240)
            else:
                # normalize
                t = fx / (maxf+1) if maxf>0 else 0
                blue = 200 + int((1 - t) * 55)
                color = (200 - int(t*80), 200 - int(t*80), blue)
            pygame.draw.rect(screen, color, (sx, sy, CELL, CELL))
            # optional flood number
            if show_nums and fx < 999:
                txt = font.render(str(fx), True, (0,0,0))
                screen.blit(txt, (sx + 3, sy + 3))
            # draw cell border faint
            pygame.draw.rect(screen, (180,180,180), (sx,sy,CELL,CELL), 1)
            # draw walls
            # note: walls bits: N=1, E=2, S=4, W=8
            if walls[x][y] & W_NORTH:
                pygame.draw.line(screen, (0,0,0), (sx, sy), (sx + CELL, sy), 3)
            if walls[x][y] & W_EAST:
                pygame.draw.line(screen, (0,0,0), (sx + CELL, sy), (sx + CELL, sy + CELL), 3)
            if walls[x][y] & W_SOUTH:
                pygame.draw.line(screen, (0,0,0), (sx, sy + CELL), (sx + CELL, sy + CELL), 3)
            if walls[x][y] & W_WEST:
                pygame.draw.line(screen, (0,0,0), (sx, sy), (sx, sy + CELL), 3)
    # draw center area
    for (cx,cy) in center_coords:
        sx, sy = cell_to_screen(cx,cy)
        pygame.draw.rect(screen, (180,255,180), (sx+4, sy+4, CELL-8, CELL-8))
    # draw astar path if exists
    if astar_path:
        for (px,py) in astar_path:
            sx, sy = cell_to_screen(px,py)
            pygame.draw.rect(screen, (255,220,120), (sx+6, sy+6, CELL-12, CELL-12))
    # draw mouse
    mx, my = cell_to_screen(curr_x, curr_y)
    cx_pix = mx + CELL//2
    cy_pix = my + CELL//2
    pygame.draw.circle(screen, (200,50,50), (cx_pix, cy_pix), CELL//4)
    # direction arrow
    if Direction == 0:
        arrow = [(cx_pix, cy_pix - 8), (cx_pix-6, cy_pix+4), (cx_pix+6, cy_pix+4)]
    elif Direction == 1:
        arrow = [(cx_pix + 8, cy_pix), (cx_pix-4, cy_pix-6), (cx_pix-4, cy_pix+6)]
    elif Direction == 2:
        arrow = [(cx_pix, cy_pix + 8), (cx_pix-6, cy_pix-4), (cx_pix+6, cy_pix-4)]
    else:
        arrow = [(cx_pix - 8, cy_pix), (cx_pix+4, cy_pix-6), (cx_pix+4, cy_pix+6)]
    pygame.draw.polygon(screen, (0,0,0), arrow)

    # HUD
    hud = [
        "SPACE: step_to_lowest   G: toggle auto run   A: A* path follow   R: regen maze   V: toggle flood numbers",
        f"pos=({curr_x},{curr_y}) dir={Direction}  auto={auto_run}  astar_follow={follow_astar}"
    ]
    for i, line in enumerate(hud):
        txt = font.render(line, True, (10,10,10))
        screen.blit(txt, (6, 6 + i*18))

    pygame.display.flip()

# ---------- Initialize / reset ----------
def reset():
    global curr_x, curr_y, Direction, auto_run, astar_path, follow_astar
    generate_maze()
    # set initial mouse location (corner like Arduino start 0,0)
    curr_x, curr_y = 0, 0
    Direction = 0
    auto_run = False
    follow_astar = False
    astar_path = []
    flood_fill()

# ---------- Main loop ----------
def main():
    global auto_run, show_nums, curr_x, curr_y, Direction, follow_astar, astar_path
    reset()
    running = True
    step_timer = 0
    step_interval = 150  # ms per auto step
    while running:
        dt = clock.tick(FPS)
        step_timer += dt
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                    break
                if event.key == pygame.K_SPACE:
                    # recompute flood then single step
                    flood_fill()
                    step_to_lowest()
                if event.key == pygame.K_g:
                    auto_run = not auto_run
                    follow_astar = False
                    astar_path = []
                if event.key == pygame.K_v:
                    show_nums = not show_nums
                if event.key == pygame.K_r:
                    reset()
                if event.key == pygame.K_b:  # زرار B
                 curr_x, curr_y = 0, 0   # نرجع للبداية
                 Direction = 0            # إعادة الاتجاه للبداية
                    # لو عايز تحدث الفلود
                 flood_fill()
                      # لو عايز توقف أي حركة آلية
                 #auto_run = False
                 #follow_astar = False
                 #astar_path = []    
                if event.key == pygame.K_a:
                    # compute A* from current to nearest center cell (choose center 0,0 index)
                    # here choose center_coords[0] as representative; better choose nearest center cell
                    # find nearest center by manhattan
                    best_c = min(center_coords, key=lambda c: heuristic((curr_x,curr_y), c))
                    path = astar((curr_x,curr_y), best_c)
                    if path:
                        astar_path = path[1:]  # exclude current
                        follow_astar = True
                        auto_run = False
                    else:
                        astar_path = []
                        follow_astar = False

        # actions
        if follow_astar and astar_path:
         # follow path with some delay
          if step_timer >= step_interval:
            step_timer = 0
            nx, ny = astar_path.pop(0)
        # compute direction to move
            for d in range(4):
                if curr_x + dx[d] == nx and curr_y + dy[d] == ny:
                    turn_to_dir(d)
                    curr_x, curr_y = nx, ny
                    break
        # تحقق إذا وصل للجول بعد الحركة
          
            if (curr_x, curr_y) in center_coords:
                follow_astar = False
                astar_path = []
                auto_run = False
                print("Goal reached! You win!")

        elif auto_run:
            if step_timer >= step_interval:
                step_timer = 0
                flood_fill()
                moved = step_to_lowest()
                if not moved:
                    auto_run = False
        if (curr_x,curr_y) in center_coords:
            print("Goal reached! You win!")
            #running=False
            auto_run = False
            

        draw()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()