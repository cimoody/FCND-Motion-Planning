from enum import Enum
from queue import PriorityQueue
import numpy as np

# Quadroter assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 3 values are the delta of the action relative
    to the current grid position. The fourth and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)
    
    NORTH_WEST = (-1, -1, 0, np.sqrt(2))
    NORTH_EAST = (-1, 1, 0, np.sqrt(2))
    SOUTH_WEST = (1, -1, 0, np.sqrt(2))
    SOUTH_EAST = (1, 1, 0, np.sqrt(2))
    
    UP_WEST = (0, -1, 1, np.sqrt(2))
    UP_EAST = (0, 1, 1, np.sqrt(2))
    UP_NORTH = (0, -1, 1, np.sqrt(2))
    UP_SOUTH = (0, 1, 1, np.sqrt(2))
    
    DOWN_WEST = (0, -1, -1, np.sqrt(2))
    DOWN_EAST = (0, 1, -1, np.sqrt(2))
    DOWN_NORTH = (0, -1, -1, np.sqrt(2))
    DOWN_SOUTH = (0, 1, -1, np.sqrt(2))
    
    UP_NORTH_WEST = (-1, -1, 1, np.sqrt(3))
    UP_NORTH_EAST = (-1, 1, 1, np.sqrt(3))
    UP_SOUTH_WEST = (1, -1, 1, np.sqrt(3))
    UP_SOUTH_EAST = (1, 1, 1, np.sqrt(3))
    
    DOWN_NORTH_WEST = (-1, -1, -1, np.sqrt(3))
    DOWN_NORTH_EAST = (-1, 1, -1, np.sqrt(3))
    DOWN_SOUTH_WEST = (1, -1, -1, np.sqrt(3))
    DOWN_SOUTH_EAST = (1, 1, -1, np.sqrt(3))

    @property
    def cost(self):
        ### Not sure if sintax is correct for python
        if self.value[3].isnull() == False:
            return self.value[3]
        else:
            return self.value[2]

    @property
    def delta(self):
        ### Not sure if sintax is correct for python
        if self.value[3].isnull() == False:
            return (self.value[0], self.value[1], self.value[2])
        else:
            return (self.value[0], self.value[1])
            


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    if grid.shape[2].isnull() == True:
        n, m = grid.shape[0] - 1, grid.shape[1] - 1
        x, y = current_node
    else:
        n, m, l = grid.shape[0] - 1, grid.shape[1] - 1, grid.shape[2] - 1
        x, y, z = current_node

    # check if the node is off the grid or
    # it's an obstacle
    if grid.shape[2].isnull() == True:
        valid_actions.remove(Action.UP_NORTH)
        valid_actions.remove(Action.UP_SOUTH)
        valid_actions.remove(Action.UP_WEST)
        valid_actions.remove(Action.UP_EAST)
    
        valid_actions.remove(Action.UP_NORTH_WEST)
        valid_actions.remove(Action.UP_NORTH_EAST)
        valid_actions.remove(Action.UP_SOUTH_WEST)
        valid_actions.remove(Action.UP_SOUTH_EAST)
        
        valid_actions.remove(Action.DOWN_NORTH)
        valid_actions.remove(Action.DOWN_SOUTH)
        valid_actions.remove(Action.DOWN_WEST)
        valid_actions.remove(Action.DOWN_EAST)
    
        valid_actions.remove(Action.DOWN_NORTH_WEST)
        valid_actions.remove(Action.DOWN_NORTH_EAST)
        valid_actions.remove(Action.DOWN_SOUTH_WEST)
        valid_actions.remove(Action.DOWN_SOUTH_EAST)
        
        if x - 1 < 0 or grid[x - 1, y] == 1:
            valid_actions.remove(Action.NORTH)
        if x + 1 > n or grid[x + 1, y] == 1:
            valid_actions.remove(Action.SOUTH)
        if y - 1 < 0 or grid[x, y - 1] == 1:
            valid_actions.remove(Action.WEST)
        if y + 1 > m or grid[x, y + 1] == 1:
            valid_actions.remove(Action.EAST)
    
        if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
            valid_actions.remove(Action.NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
            valid_actions.remove(Action.NORTH_EAST)
        if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
            valid_actions.remove(Action.SOUTH_WEST)
        if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
            valid_actions.remove(Action.SOUTH_EAST)
        
    else:
        if x - 1 < 0 or grid[x - 1, y, z] == 1:
            valid_actions.remove(Action.NORTH)
        if x + 1 > n or grid[x + 1, y, z] == 1:
            valid_actions.remove(Action.SOUTH)
        if y - 1 < 0 or grid[x, y - 1, z] == 1:
            valid_actions.remove(Action.WEST)
        if y + 1 > m or grid[x, y + 1, z] == 1:
            valid_actions.remove(Action.EAST)
    
        if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1, z] == 1:
            valid_actions.remove(Action.NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1, z] == 1:
            valid_actions.remove(Action.NORTH_EAST)
        if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1, z] == 1:
            valid_actions.remove(Action.SOUTH_WEST)
        if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1, z] == 1:
            valid_actions.remove(Action.SOUTH_EAST)

        if x - 1 < 0 or grid[x - 1, y, z + 1] == 1:
            valid_actions.remove(Action.UP_NORTH)
        if x + 1 > n or grid[x + 1, y, z + 1] == 1:
            valid_actions.remove(Action.UP_SOUTH)
        if y - 1 < 0 or grid[x, y - 1, z + 1] == 1:
            valid_actions.remove(Action.UP_WEST)
        if y + 1 > m or grid[x, y + 1, z + 1] == 1:
            valid_actions.remove(Action.UP_EAST)
            
        if (x - 1 < 0 or z - 1 < 0) or grid[x - 1, y, z - 1] == 1:
            valid_actions.remove(Action.DOWN_NORTH)
        if (x + 1 > n or z - 1 < 0) or grid[x + 1, y, z - 1] == 1:
            valid_actions.remove(Action.DOWN_SOUTH)
        if (y - 1 < 0 or z - 1 < 0) or grid[x, y - 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_WEST)
        if (y + 1 > m or z - 1 < 0) or grid[x, y + 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_EAST)
    
        if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1, z + 1] == 1:
            valid_actions.remove(Action.UP_NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1, z + 1] == 1:
            valid_actions.remove(Action.UP_NORTH_EAST)
        if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1, z + 1] == 1:
            valid_actions.remove(Action.UP_SOUTH_WEST)
        if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1, z + 1] == 1:
            valid_actions.remove(Action.UP_SOUTH_EAST)
    
        if (x - 1 < 0 or y - 1 < 0 or z - 1 < 0) or grid[x - 1, y - 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m or z - 1 < 0) or grid[x - 1, y + 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_NORTH_EAST)
        if (x + 1 > n or y - 1 < 0 or z - 1 < 0) or grid[x + 1, y - 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_SOUTH_WEST)
        if (x + 1 > n or y + 1 > m or z - 1 < 0) or grid[x + 1, y + 1, z - 1] == 1:
            valid_actions.remove(Action.DOWN_SOUTH_EAST)
        
    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = []
    # visited.append(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if len(current_node) < 2:
            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                for action in valid_actions(grid, current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)
                
                    if next_node not in visited:                
                        visited.add(next_node)               
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
        if (current_node[0] == start[0] and
            (len(current_node) > 2 )):
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


