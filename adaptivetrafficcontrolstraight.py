"""
intersection_sim_adaptive.py
Adaptive 2D birdseye 4-way intersection simulation using pygame.

Changes from user's original:
 - Integrated a simple adaptive traffic controller: green duration is computed per phase based on measured queue lengths for the approaches served by that phase.
 - Ensures vehicles that enter the intersection during green will not be stopped afterwards (they get a `passed_intersection` flag and are allowed to clear the intersection).
 - Computes per-phase dynamic duration at each phase start (clamped between MIN_GREEN and MAX_GREEN).
 - Queue measurement counts queued/stopped cars near each stop line.
 - Keeps visuals and basic design unchanged (only internal logic changes and a couple of tiny drawing fixes).

Run with Python 3 and pygame installed.
"""

import pygame
import random
import math
from collections import deque

# ---------------------
# CONFIG
# ---------------------
FPS = 30
WINDOW_W, WINDOW_H = 800, 800
LANE_WIDTH = 40
ROAD_HALF = 180
CENTER = (WINDOW_W // 2, WINDOW_H // 2)
GREEN_DURATION = 30  # default seconds per green phase (used initially)

# ---------------------
# Adaptive traffic parameters
# ---------------------
MIN_GREEN = 5       # minimum green duration (seconds)
MAX_GREEN = 20      # maximum green duration (seconds)
GAP_THRESHOLD = 2   # seconds to gap-out if no car passes


# Adaptive timing parameters
MIN_GREEN = 10   # minimum green (s)
MAX_GREEN = 60   # maximum green (s)
PER_CAR_ADDITIONAL = 3.5  # seconds added per queued car
BASE_GREEN = 10  # base green when no cars

STOP_DIST = 130
north_stop_y = CENTER[1] - STOP_DIST
south_stop_y = CENTER[1] + STOP_DIST
west_stop_x  = CENTER[0] - STOP_DIST
east_stop_x  = CENTER[0] + STOP_DIST

SENSOR_RANGE = 300
SENSOR = { 'N': (north_stop_y - SENSOR_RANGE, north_stop_y) }

# phases: list of dicts, each dict:
#  - 'green': set of directions in green ('N','S','E','W')
#  - 'barriers': list of barrier positions as tuples like ('N','leftturn') or simple direction e.g. 'N'
PHASES = [
    {'name': 'N-only', 'green': {'N'}, 'barriers': [('S','right'), ('W','right'),('E','right')]},
    {'name': 'S-only', 'green': {'S'}, 'barriers': [('N','right'), ('W','right'),('E','right')]},
    {'name': 'W-only', 'green': {'W'}, 'barriers': [('N','right'), ('E','right'),('S','right')]},
    {'name': 'E-only', 'green': {'E'}, 'barriers': [('W','right'), ('S','right'),('N','right')]},
]

# ---------------------
# Pygame setup
# ---------------------
pygame.init()
screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
clock = pygame.time.Clock()
font = pygame.font.SysFont('Arial', 22)

# Colors
ROAD = (50, 50, 50)
LINE = (200, 200, 200)
GREEN = (0, 200, 0)
RED = (200, 0, 0)
YELLOW = (220, 200, 20)
WHITE = (255, 255, 255)
CAR_COLORS = [(220, 50, 50), (50, 50, 220), (50, 180, 50), (200, 120, 20)]
BLACK = (0, 0, 0)
# ---------------------
# Helper geometry
# ---------------------

def rect_centered(x,y,w,h):
    return pygame.Rect(int(x-w/2), int(y-h/2), w, h)

# ---------------------
# Car class
# ---------------------
class Car:
    def __init__(self, direction, route='straight'):
        self.direction = direction  # N,S,E,W
        self.route = route  # 'straight'/'left'/'right' - for visualization only
        self.speed = 2.2 + random.random() * 0.8  # pixels per frame
        self.color = random.choice(CAR_COLORS)
        self.size = (18, 10)
        # spawn position based on direction
        if direction == 'N':
            self.x = CENTER[0] + LANE_WIDTH / 2  # right lane southbound
            self.y = -30
            self.vx, self.vy = 0, self.speed
        elif direction == 'S':
            self.x = CENTER[0] - LANE_WIDTH / 2  # right lane northbound
            self.y = WINDOW_H + 30
            self.vx, self.vy = 0, -self.speed
        elif direction == 'E':
            self.x = WINDOW_W + 30
            self.y = CENTER[1] + LANE_WIDTH / 2  # lower lane westbound
            self.vx, self.vy = -self.speed, 0
        else:  # W
            self.x = -30
            self.y = CENTER[1] - LANE_WIDTH / 2  # upper lane eastbound
            self.vx, self.vy = self.speed, 0
        self.stopped = False
        self.passed_intersection = False  # becomes True once car has entered/cleared intersection

    def update(self):
        if not self.stopped:
            self.x += self.vx
            self.y += self.vy

    def draw(self, surf):
        w,h = self.size
        rect = pygame.Rect(0,0,w,h)
        rect.center = (int(self.x), int(self.y))
        pygame.draw.rect(surf, self.color, rect)

    def distance_to_center(self):
        return math.hypot(self.x - CENTER[0], self.y - CENTER[1])

# ---------------------
# Intersection visuals
# ---------------------
def draw_intersection(surface):
    # background
    surface.fill((135, 206, 235))  # grass
    # roads: vertical and horizontal
    pygame.draw.rect(surface, ROAD, rect_centered(WINDOW_W//2, WINDOW_H//2, LANE_WIDTH*4 + 100, ROAD_HALF*2+500))
    pygame.draw.rect(surface, ROAD, rect_centered(WINDOW_W//2, WINDOW_H//2, ROAD_HALF*2+500, LANE_WIDTH*4 + 100))
    # lane markings (vertical and horizontal)
    for i in range(-2,3):
        x = CENTER[0] + i*LANE_WIDTH/2
        if i == 0:
            color = (255, 255, 0) # yellow for middle
            width = 3
            # dashed vertical line
            dash_length = 20
            gap_length = 20
            y = 0
            while y < WINDOW_H:
                pygame.draw.line(surface, color, (x, y), (x, min(y + dash_length, WINDOW_H)), width)
                y += dash_length + gap_length
            else:
                color = (255, 255, 255)
                width = 5
            pygame.draw.line(surface, color, (x, 0), (x, WINDOW_H), width)
        elif i == 1:
            color = (50, 50, 50,0)
            width = 3
        elif i == -1:
            color = (50, 50, 50,0)
            width = 3
        else :
            color = (255, 255, 0)
            width = 3
        # vertical lines
        pygame.draw.line(surface,color, start_pos=(x,0),end_pos=(x,WINDOW_H), width=width)
        # horizontal lines
        y = CENTER[1] + i*LANE_WIDTH/2
        pygame.draw.line(surface,color, start_pos=(0,y),end_pos=(WINDOW_W,y), width=width)

    # center box outline
    pygame.draw.rect(surface, (0,0,0), rect_centered(CENTER[0], CENTER[1], LANE_WIDTH*4, LANE_WIDTH*4), 2)

# ---------------------
# Light and barrier drawing
# ---------------------
def draw_lights(surface, phase):
    # For each direction draw a light (a small circle near intersection)
    offsets = {
        'N': (CENTER[0] + LANE_WIDTH*1.5, CENTER[1] - ROAD_HALF + 75),
        'S': (CENTER[0] - LANE_WIDTH*1.5, CENTER[1] + ROAD_HALF - 75),
        'W': (CENTER[0] - ROAD_HALF + 75, CENTER[1] - LANE_WIDTH*1.5),
        'E': (CENTER[0] + ROAD_HALF - 75, CENTER[1] + LANE_WIDTH*1.5)
    }
    for d, pos in offsets.items():
        col = GREEN if d in phase['green'] else RED
        pygame.draw.circle(surface, col, pos, 12)

    # draw barriers (yellow) as short rectangles near intersection based on phase['barriers']
    for b in phase.get('barriers', []):
        dirn = b[0]
        kind = b[1] if len(b) > 1 else 'block'
        if dirn == 'N':
            bx = CENTER[0] - 35 if kind == 'left' else CENTER[0] + 35
            by = CENTER[1] - LANE_WIDTH*2
            pygame.draw.rect(surface, RED, pygame.Rect(bx-35, by-6, 40, 12))
        elif dirn == 'S':
            bx = CENTER[0] + 35 if kind == 'left' else CENTER[0] - 35
            by = CENTER[1] + LANE_WIDTH*2
            pygame.draw.rect(surface, RED, pygame.Rect(bx-6, by-6, 40, 12))
        elif dirn == 'E':
            bx = CENTER[0] + LANE_WIDTH*2
            by = CENTER[1] - 35 if kind == 'left' else CENTER[1] + 35
            pygame.draw.rect(surface, RED, pygame.Rect(bx-6, by-35, 12, 40))
        elif dirn == 'W':
            bx = CENTER[0] - LANE_WIDTH*2
            by = CENTER[1] + 35 if kind == 'left' else CENTER[1] - 35
            pygame.draw.rect(surface, RED, pygame.Rect(bx-6, by-6, 12, 40))

# ---------------------
# Simulation controller
# ---------------------
cars = []
car_spawn_timer = 0.0
phase_index = 0
phase_elapsed = 0.0
current_phase_duration = GREEN_DURATION


def spawn_car():
    d = random.choice(['N','S','E','W'])
    r = random.choice(['straight','left','right'])
    cars.append(Car(d, r))


def measure_queues():
    """Return a dict with queue lengths (number of stopped cars near stop line) per direction."""
    queues = {'N':0,'S':0,'E':0,'W':0}
    for c in cars:
        # consider cars within a region approaching the intersection
        if c.direction == 'N' and c.y < north_stop_y + 20:
            if c.stopped or c.distance_to_center() < 160:
                queues['N'] += 1
        elif c.direction == 'S' and c.y > south_stop_y - 20:
            if c.stopped or c.distance_to_center() < 160:
                queues['S'] += 1
        elif c.direction == 'W' and c.x < west_stop_x + 20:
            if c.stopped or c.distance_to_center() < 160:
                queues['W'] += 1
        elif c.direction == 'E' and c.x > east_stop_x - 20:
            if c.stopped or c.distance_to_center() < 160:
                queues['E'] += 1
    return queues


def compute_phase_duration(phase, queues):
    """Compute adaptive green duration for a phase based on queues in directions served by that phase."""
    demand = 0
    for d in phase['green']:
        demand += queues.get(d,0)
    duration = BASE_GREEN + PER_CAR_ADDITIONAL * demand
    duration = max(MIN_GREEN, min(MAX_GREEN, duration))
    return duration


def apply_controls_to_cars(phase):
    blocked_dirs = {b[0] for b in phase.get('barriers', [])}

    # Group cars by lane (direction)
    cars_by_dir = {'N': [], 'S': [], 'E': [], 'W': []}
    for car in cars:
        cars_by_dir[car.direction].append(car)

    # Sort cars in each lane by distance to center (front-most first)
    for d in cars_by_dir:
        if d in ['N', 'S']:
            # For northbound (N) moving down (+y), larger y means closer to intersection
            cars_by_dir[d].sort(key=lambda c: c.y, reverse=(d == 'N'))
        else:
            # For westbound (W) moving right (+x), larger x means closer
            cars_by_dir[d].sort(key=lambda c: c.x, reverse=(d == 'W'))

    # apply stop logic
    for d, lane_cars in cars_by_dir.items():
        for i, car in enumerate(lane_cars):
            # Is the lane green and not blocked?
            green = (car.direction in phase['green'] and car.direction not in blocked_dirs)

            # Default: allow moving
            # However, once a car has "passed_intersection" we never stop it (it must clear)
            if car.passed_intersection:
                car.stopped = False
                continue

            car.stopped = False

            # Stop if red & near intersection
            # Determine proximity to intersection by direction
            near = False
            if car.direction == 'N':
                near = car.y > north_stop_y - 40
            elif car.direction == 'S':
                near = car.y < south_stop_y + 40
            elif car.direction == 'W':
                near = car.x > west_stop_x - 40
            elif car.direction == 'E':
                near = car.x < east_stop_x + 40

            if not green and near:
                car.stopped = True

            # Car-following: if too close to car ahead, stop (but don't stop the front car)
            if i > 0:
                front_car = lane_cars[i - 1]
                dx = car.x - front_car.x
                dy = car.y - front_car.y
                dist = (dx**2 + dy**2) ** 0.5
                if dist < 25:
                    car.stopped = True

            # If the lane is green and this car is near/over the stop line, allow it to mark as passed and not be stopped
            if green:
                # mark as passed_intersection once car crosses the center line in its travel direction
                if car.direction == 'N' and car.y > CENTER[1]:
                    car.passed_intersection = True
                elif car.direction == 'S' and car.y < CENTER[1]:
                    car.passed_intersection = True
                elif car.direction == 'W' and car.x > CENTER[0]:
                    car.passed_intersection = True
                elif car.direction == 'E' and car.x < CENTER[0]:
                    car.passed_intersection = True

last_car_pass = [0.0 for _ in PHASES]  # one per phase

# ---------------------
# Main loop
# ---------------------
running = True
accum_seconds = 0.0

# initialize first phase duration based on initial measured queues
queues = measure_queues()
current_phase_duration = compute_phase_duration(PHASES[phase_index], queues)

while running:
    dt = clock.tick(FPS) / 1000.0
    accum_seconds += dt
    car_spawn_timer += dt
    phase_elapsed += dt

    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        elif ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_ESCAPE:
                running = False
            if ev.key == pygame.K_RIGHT:
                # advance a phase manually
                phase_index = (phase_index + 1) % len(PHASES)
                phase_elapsed = 0.0
                # recompute duration on manual advance
                queues = measure_queues()
                current_phase_duration = compute_phase_duration(PHASES[phase_index], queues)

    # spawn cars occasionally
    if car_spawn_timer > 0.8:
        if random.random() < 0.6:
            spawn_car()
        car_spawn_timer = 0.0


        # --- adaptive gap-out logic ---
        # Count cars that are waiting (not yet passed intersection)
        def queue_length(dir):
            return sum(1 for c in cars if not c.stopped and c.direction == dir and c.distance_to_center() < 120)


        # determine current time
        now = accum_seconds

        # update last_car_pass if any car just passed the stop
        for c in cars:
            if c.distance_to_center() > 0 and not c.stopped:
                last_car_pass[phase_index] = now

        # switch phase conditions
        if phase_elapsed >= MIN_GREEN:
            # 1) queue empty for all green directions
            if all(queue_length(d) == 0 for d in current_phase['green']):
                phase_index = (phase_index + 1) % len(PHASES)
                phase_elapsed = 0.0
            # 2) gap threshold: no car passed recently
            elif now - last_car_pass[phase_index] >= GAP_THRESHOLD:
                phase_index = (phase_index + 1) % len(PHASES)
                phase_elapsed = 0.0
            # 3) max green exceeded
            elif phase_elapsed >= MAX_GREEN:
                phase_index = (phase_index + 1) % len(PHASES)
                phase_elapsed = 0.0

        # measure queues and compute next phase duration
        queues = measure_queues()
        current_phase_duration = compute_phase_duration(PHASES[phase_index], queues)

    current_phase = PHASES[phase_index]
    countdown = max(0, int(math.ceil(current_phase_duration - phase_elapsed)))

    # apply controls to cars (sets stopped flags / passed_intersection)
    apply_controls_to_cars(current_phase)

    # update cars
    for c in list(cars):
        c.update()
        # remove if far beyond screen
        if c.x < -150 or c.x > WINDOW_W + 150 or c.y < -150 or c.y > WINDOW_H + 150:
            cars.remove(c)

    # draw
    draw_intersection(screen)
    draw_lights(screen, current_phase)

    # draw cars
    for c in cars:
        c.draw(screen)

    # overlay countdown and phase name
    txt = font.render(f"Time left: {countdown}s", True, GREEN)
    screen.blit(txt, (20, 90))

    # small legend and instructions
    instr = font.render("ESC to quit.", True, WHITE)
    screen.blit(instr, (20, 50))

    # optional: show queue counts (debug)
    q = measure_queues()
    qtxt = font.render(f"Queues N:{q['N']} S:{q['S']} E:{q['E']} W:{q['W']}", True, WHITE)
    screen.blit(qtxt, (20, 120))

    pygame.display.flip()

pygame.quit()

