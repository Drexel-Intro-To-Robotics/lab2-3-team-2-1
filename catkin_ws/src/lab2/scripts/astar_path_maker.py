#!/usr/bin/env python3

import rospy
import heapq
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import GridCells

class AStarPathMaker:
    # Setup
    def __init__(self):
        rospy.init_node('astar_path_maker')

        # map state
        self.map = None
        self.map_info = None # rez, origin, w, h
        self.start = None
        self.goal = None

        # Subscribe to created map
        rospy.Subscriber('/map', OccupancyGrid,self.map_cb)
        # Subscribe to initial pose
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_cb)
        # Subscribe to move_base
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

        # Publishers - called in rviz (part 3)
        self.pub_frontier = rospy.Publisher('/frontier_cells', GridCells, queue_size=10)
        self.pub_expanded = rospy.Publisher('/expanded_cells', GridCells, queue_size=10)
        self.pub_path_cells = rospy.Publisher('/path_cells', GridCells, queue_size=10, latch=True)
        self.pub_nav_path = rospy.Publisher('/planned_path', Path, queue_size=10, latch=True)
        self.pub_raw_path = rospy.Publisher('/raw_path_cells', GridCells, queue_size=10, latch=True)
        self.pub_raw_nav  = rospy.Publisher('/raw_planned_path', Path, queue_size=10, latch=True)

        rospy.spin()

    #------------------Callback functions----------------
    # map
    def map_cb(self, msg):
        # get info
        self.map_info = msg.info
        # save info
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # called when 2d pose estimate is used
    def start_cb(self, msg):
        self.start = self.world_to_grid(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    # 2d nav goal
    def goal_cb(self, msg):
        self.goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        if self.start is not None and self.map is not None:
            self.astar()


    #--------------Coordinate Functions---------------
    # meters --> grid
    def world_to_grid(self, wx, wy):
        # resolution
        res = self.map_info.resolution
        # origin
        origin = self.map_info.origin.position
        # convert
        col = int((wx - origin.x) / res)
        row = int((wy - origin.y) / res)
        return (col, row)

    # grid --> meters
    def grid_to_world(self, col, row):
        res = self.map_info.resolution
        origin = self.map_info.origin.position
        wx = origin.x + (col + 0.5) * res
        wy = origin.y + (row + 0.5) * res
        return (wx, wy)

    # check if neighbor cell is free
    def is_free(self, col, row):
        w = self.map_info.width
        h = self.map_info.height
        # check bounds
        if col < 0 or row < 0 or col >= w or row >= h:
            return False
        return self.map[row][col] == 0

    def make_grid(self, cells):
        #create empty grid
        msg = GridCells()
        # tell rviz grid is in map
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        # size of cells (m)
        msg.cell_width = self.map_info.resolution
        msg.cell_height = self.map_info.resolution
        for (col, row) in cells:
            # convert
            wx, wy = self.grid_to_world(col, row)
            p = Point()
            p.x, p.y, p.z = wx, wy, 0.0
            # add point
            msg.cells.append(p)
        return msg

    # same as make grid but z is raised to make sure it shows
    def make_path_grid(self, cells):
        msg = GridCells()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.cell_width  = self.map_info.resolution
        msg.cell_height = self.map_info.resolution
        for (col, row) in cells:
            wx, wy = self.grid_to_world(col, row)
            p = Point()
            p.x, p.y, p.z = wx, wy, 0.05  # slightly above other cells
            msg.cells.append(p)
        return msg
    
    #------------------- Main A* algorithm----------------------
    def astar(self):
        # setup
        start, goal = self.start, self.goal
        # heuristic - euclidean
        def euclidean(a, b):
            return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

        # priority queue
        queue = []
        heapq.heappush(queue, (euclidean(start, goal), 0.0, start))

        # cells for calcs
        came_from = {start: None}
        g_score = {start: 0.0}
        frontier = set([start])
        expanded = set()

        # neighbor directions - 8 directions
        neighbors = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        rate = rospy.Rate(20)

        # main loop
        while queue:
            # pop lowest scored node
            f, g, current = heapq.heappop(queue)
            if current in expanded:
                continue
            
            frontier.discard(current)
            expanded.add(current)

            # check if node is goal - done
            if current == goal:
                rospy.loginfo("Goal found!")
                path = self.reconstruct_path(came_from, goal)
                self.publish_results(path)
                return

            # goal not found - check neighbors
            for col, row in neighbors:
                #get grid coordinates
                nb = (current[0]+col, current[1]+row)
                # skip if occupied cell (wall)
                if not self.is_free(*nb) or nb in expanded:
                    continue
                # movement cost (diag = sqrt(2))
                step = np.sqrt(2) if col != 0 and row != 0 else 1.0
                # temp g score
                tg = g + step
                #check if this node is a better path
                if tg < g_score.get(nb, float('inf')):
                    # used for reconstructing path
                    came_from[nb] = current
                    # save new best cost
                    g_score[nb] = tg
                    # add neighbor to queue
                    heapq.heappush(queue, (tg + euclidean(nb, goal), tg, nb))
                    # add frontier for rviz
                    frontier.add(nb)
            # Show the search
            self.pub_frontier.publish(self.make_grid(list(frontier)))
            self.pub_expanded.publish(self.make_grid(list(expanded)))
            rate.sleep()

    # build path (raw and optimized)
    def reconstruct_path(self, came_from, goal):
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()

        # Publish raw path first
        self.pub_raw_path.publish(self.make_grid(path))
        self.publish_nav_path(path, self.pub_raw_nav)

        # Optimize — smooth corners first, then line of sight
        smoothed = self.smooth_corners(path)
        optimized = self.optimize_path(smoothed)

        # Log comparison — both variables exist here now
        rospy.loginfo("-----------Path Comparison---------------")
        rospy.loginfo("Raw waypoints: %d", len(path))
        rospy.loginfo("Optimized waypoints: %d", len(optimized))
        rospy.loginfo("Reduction: .1f%%",
                    100.0 * (1 - len(optimized) / len(path)))
        return optimized
    
    # remove points on straight line (redundant)
    def optimize_path(self, path):
        if len(path) < 3:
            return path
        
        optimized = [path[0]]
        anchor = 0

        i = 2
        while i < len(path):
            # Skip path if you can see another unobstructed
            if self.line_of_sight(path[anchor], path[i]):
                i += 1  # keep looking
            else:
                # Can't skip path[i-1] --> necessary waypoint
                optimized.append(path[i-1])
                anchor = i - 1
                i += 1
        optimized.append(path[-1])  # always keep goal
        return optimized
    
    # line of sight calculator (Brensenham's)
    def line_of_sight(self, a,b):
        # start cell
        c0, r0 = a
        # end cell
        c1, r1 = b
        # total column, row distance
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        #step direction
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        # error
        err = dc - dr

        while True:
            if not self.is_free_inflated(c0, r0):
                # hit obstacle
                return False
            if c0 == c1 and r0 == r1:
                # reached destination
                return True
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c0  += sc
            if e2 < dc:
                err += dc
                r0  += sr
    
    # check cell and neighbors - give a 0.2m birth
    def is_free_inflated(self, col, row, inflation=4):
        for dc in range(-inflation, inflation+1):
            for dr in range(-inflation, inflation+1):
                if not self.is_free(col+dc, row+dr):
                    return False
        return True
    
    # smooth over areas with corners (obstacles)
    def smooth_corners(self, path):
        if len(path) < 3:
            return path

        smoothed = [path[0]]

        for i in range(1, len(path) - 1):
            prev = path[i-1]
            curr = path[i]
            nxt = path[i+1]

            # Vectors into and out of the corner
            v_in = (curr[0] - prev[0], curr[1] - prev[1])
            v_out = (nxt[0]  - curr[0], nxt[1]  - curr[1])

            # Normalize
            len_in  = max(np.sqrt(v_in[0]**2  + v_in[1]**2),  1e-6)
            len_out = max(np.sqrt(v_out[0]**2 + v_out[1]**2), 1e-6)
            v_in_n  = (v_in[0]  / len_in,  v_in[1]  / len_in)
            v_out_n = (v_out[0] / len_out, v_out[1] / len_out)

            # Dot product
            dot = v_in_n[0]*v_out_n[0] + v_in_n[1]*v_out_n[1]

            # Only smooth sharp turns (greater than 60 degrees)
            if dot < 0.5:
                OFFSET = 4  # cells before corner

                # Approach point — back off from corner along incoming vector
                approach = (
                    int(curr[0] - v_in_n[0] * OFFSET),
                    int(curr[1] - v_in_n[1] * OFFSET)
                )
                # Exit point — pull away from corner along outgoing vector
                exit_pt = (
                    int(curr[0] + v_out_n[0] * OFFSET),
                    int(curr[1] + v_out_n[1] * OFFSET)
                )

                # Only add if both points are free
                if self.is_free_inflated(*approach):
                    smoothed.append(approach)
                smoothed.append(curr)  # keep the corner itself
                if self.is_free_inflated(*exit_pt):
                    smoothed.append(exit_pt)
            else:
                smoothed.append(curr)

        smoothed.append(path[-1])
        return smoothed
    
    # used to show path in rviz
    def publish_nav_path(self, path, publisher):
        nav_path = Path()
        nav_path.header.frame_id = 'map'
        nav_path.header.stamp = rospy.Time.now()
        for (col, row) in path:
            wx, wy = self.grid_to_world(col, row)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)
        publisher.publish(nav_path)

    # publish results - optimal path and original path
    def publish_results(self, path):
        self.pub_path_cells.publish(self.make_path_grid(path))
        self.publish_nav_path(path, self.pub_nav_path)

if __name__ == '__main__':
    AStarPathMaker()