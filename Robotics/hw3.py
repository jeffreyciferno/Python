import math
import random
import matplotlib.pyplot as plt

class Point:

    def __init__(self, x=0, y=0, obs=False, root=None):
        self.x = x
        self.y = y
        self.root = root
        self.child = []
        self.obs = obs

    def equals(self, point):
        if self.x == point.x and self.y == point.y:
            return True
        return False

    def show(self):
        print(f"({self.x}, {self.y})")

class Line:
    def __init__(self, a, b, obs):
        self.a = Point(a.x, a.y)
        self.b = Point(b.x, b.y)
        self.obstacle = obs

    def length(self):
        x = self.b.x - self.a.x
        y = self.b.y - self.a.y

        return math.sqrt(x**2, y**2)

class World:

    graph = []

    def __init__(self, max_X, max_Y):
        self.max_X = max_X
        self.max_Y = max_Y

    def _find_point(self, point):
        for i, p in enumerate(self.graph):
            if p.equals(point):
                return i
        return -1
    
    def add_point(self, point):
        self.graph.append(point)

    def add_line(self, pa, pb):
        pb.root = pa
        index = self._find_point(pa)
        if index != -1:
            self.graph[index].child.append(pb)
        else:
            pa.child.append(pb)
            self.graph.append(pa)
        
        index = self._find_point(pb)

        if index == -1:
            self.graph.append(pb)

    
    def _find_orientation(self, a, b, c):
        val = (float(b.y - a.y) * float(c.x - b.x)) - (float(b.x - a.x) * float(c.y - b.y))

        if val > 0: # Clockwise direction
            return 1
        elif val < 0: # Anti-clockwise direction
            return 2
        else: # Colinear
            return 0

    def _check_on_line_seg(self, a, b, c):
        if (b.x <= max(a.x, c.x) and b.x >= min(a.x, c.x)) and (b.y <= max(a.y,c.y) and b.y >= min(a.y, c.y)):
            return True

        return False

    def _check_collision(self, ab, cd):
       
        # We are finding collisions through the orientations of the points of
        # the line segment
        # Check https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ for more info

        o1 = self._find_orientation(ab.a, ab.b, cd.a)
        o2 = self._find_orientation(ab.a, ab.b, cd.b)
        o3 = self._find_orientation(cd.a, cd.b, ab.a)
        o4 = self._find_orientation(cd.a, cd.b, ab.b)

        # Normal Case
        if o1 != o2 and o3 != o4:
            return True

        # Edge cases for checking co-linear line segments
        if o1 == 0 and self._check_on_line_seg(ab.a, ab.b, cd.a):
            return True
        elif o2 == 0 and self._check_on_line_seg(ab.a, cd.b, cd.a):
            return True
        elif o3 == 0 and self._check_on_line_seg(ab.b, ab.a, cd.a):
            return True
        elif o4 == 0 and self._check_on_line_seg(ab.b, cd.a, cd.b):
            return True
        else:
            return False

    def check_collision(self, line):
        # check if line collides with any in graph
        # This can be optimized, but not a priority
        for p in self.graph:
            if p.obs == True:
                for l_p in p.child:
                    l = Line(p, l_p, False)
                    if self._check_collision(line, l) == True:
                        return True
        return False

    def _find_dist(self, p_a, p_b):
        x = p_b.x - p_a.x
        y = p_b.y - p_a.y

        return math.sqrt(x**2 + y**2)

    def find_closest_node(self, point):
        min_dist = 10000
        closest_point = None
        for p in self.graph:
            if p.obs == False:
                dist = self._find_dist(p, point)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = p

        return closest_point

    def dump_data(self):
        for p in self.graph:
            if p.root != None:
                print(f"({p.x}, {p.y}) <-- ({p.root.x}, {p.root.y})")
            else:
                print(f"({p.x}, {p.y})")
            
            for p_c in p.child:
                print(f"\t({p_c.x}, {p_c.y})")

    def random_pos(self):
        x = random.randint(0, self.max_X)
        y = random.randint(0, self.max_Y)
        return Point(x, y, False)

    def arrayfy(self, path):
        array = [[0 for y in range(self.max_Y)] for x in range(self.max_X)]

        for p in self.graph:
            if p.obs == True:
                array[p.y][p.x] = 1
        for p in path:
            array[p.y][p.x] = 1

        return array


def create_obstacles(world):
    o = Point(0, 0, True)
    a = Point(0, 499, True)
    b = Point(499, 499, True)
    c = Point(499, 0, True)

    d = Point(250, 150, True)
    e = Point(250, 350, True)

    f = Point(125, 100, True)
    g = Point(125, 200, True)

    h = Point(125, 300, True)
    i = Point(125, 400, True)
    
    j = Point(400, 100, True)
    k = Point(400, 200, True)

    l = Point(400, 300, True)
    m = Point(400, 400, True)

    world.add_line(o, a)
    world.add_line(o, c)
    world.add_line(a, b)
    world.add_line(b, c)
    world.add_line(d, e)
    world.add_line(f, g)
    world.add_line(h, i)
    world.add_line(j, k)
    world.add_line(l, m)
    
    return world

def rrt(world, start=Point(1, 1), end=Point(100, 300)):
    ## rrt pseudo code citation
    # https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
    # 
    goal = end
    world.add_point(start)
    i = 0
    errors = 10
    iter_limit = 100000
    world = World(500, 500)
    world = create_obstacles(world)
    start_line_flag = False
    while i < iter_limit:
        i += 1
        if i % 100000 == 0:
            print(f"Passed iteration {i}, graph: {len(world.graph)}")
        new_p = world.random_pos()
        if start_line_flag == False:
            nearest_p = start
        else:
            nearest_p = world.find_closest_node(new_p)
        line = Line(nearest_p, new_p, False)
        if world.check_collision(line) == True:
            continue
        start_line_flag = True
        world.add_line(nearest_p, new_p)
        if (new_p.x >= (goal.x-errors) and new_p.x <= (goal.x+errors)) \
                and (new_p.y >= (goal.y-errors) and new_p.y <= (goal.y+errors)):
            print(f"path found")
            return world, new_p, True
    return world, new_p, False

def get_path(world, start=Point(1,1), end=Point(100, 300)):
    path = []
    temp = end
    while temp.equals(start) != True:
        path.append(temp)
        temp = temp.root

    path.append(start)
    return world, path

def draw_world(world, path):
    array = world.arrayfy(path)
    obs_x = []
    obs_y = []
    for p in world.graph:
        if p.obs == True and len(p.child) != 0:
            obs_x.append(p.x)
            obs_y.append(p.y)
            for p_c in p.child:
                obs_x.append(p_c.x)
                obs_y.append(p_c.y)
                plt.plot(obs_x, obs_y, '-r')
                del obs_x[-1]
                del obs_y[-1]
            obs_x.clear()
            obs_y.clear()
    x = []
    y = []
    for p in path:
        x.append(p.x)
        y.append(p.y)
    plt.plot(x, y, '-o')
    plt.plot(obs_x, obs_y, '-r')
    plt.pcolormesh(array)
    plt.axes().set_aspect('equal')
    plt.xticks([])
    plt.yticks([])
    plt.show()

def main():
    world = World(500, 500)

    start = Point(50, 25)
    end = Point(350, 250)
    world, goal, succ = rrt(world, start=start, end=end)

    if succ:
        _, path = get_path(world, start=start, end=goal)
        for p in path:
            p.show()
        draw_world(world, path)


if __name__ == "__main__":
    main()
