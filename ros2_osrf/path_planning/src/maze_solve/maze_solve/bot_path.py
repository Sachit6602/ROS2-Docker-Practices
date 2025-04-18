import cv2
import numpy as np

from . import config

class bot_pathplanner():

    def __init__(self):
        self.DFS = DFS()

    @staticmethod
    def cords_to_pts(cords):
        # Handle both tuple coordinates and single integers
        return [cord[::-1] if isinstance(cord, (tuple, list)) else (cord, cord) for cord in cords]
    
    def draw_path_on_maze(self,maze,shortest_path_pts,method):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,shortest_path_pts[i] , shortest_path_pts[i+1], color,3)

        img_str = "maze (Found Path) [" +method +"]"
        if config.debug and config.debug_pathplanning:
            cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
            cv2.imshow(img_str, maze_bgr)

        if method == "dijisktra":
            self.dijisktra.shortest_path_overlayed = maze_bgr
        elif method == "a_star":
            self.astar.shortest_path_overlayed = maze_bgr
            
        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self,graph,start,end,maze,method = "DFS"):

        if method=="DFS":
            paths = self.DFS.get_paths(graph, start, end)
            path_to_display = paths[0]

        # Convert coordinates from (row,col) to (x,y) format
        pathpts_to_display = self.cords_to_pts(path_to_display)
        print("Found Path_pts = {}".format(pathpts_to_display))
        self.draw_path_on_maze(maze, pathpts_to_display, method)
        cv2.waitKey(0)

class DFS():

    # A not so simple problem, 
    # a recursive approach
    @staticmethod
    def get_paths(graph,start,end,path = []):

        # Update the path to where ever you have been to
        path = path + [start]

        # 2) Define the simplest case
        if (start == end):
            return [path]

        # Handle boundary case [start not part of graph]
        if start not in graph.keys():
            return []
        # List to store all possible paths from start to end
        paths = []

        # Breakdown the complex into simpler subproblems
        for node in graph[start].keys():
             #  Recursively call the problem with simpler case
            # 3) Once encountered base cond >> Roll back answer to solver subproblem
            # Checking if not already traversed and not a "case" key
            if ( (node not in path) and (node!="case") ):
                new_paths = DFS.get_paths(graph,node,end,path)
                for p in new_paths:
                    paths.append(p)
        
        return paths