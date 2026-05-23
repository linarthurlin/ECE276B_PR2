
# priority queue for OPEN list
from pqdict import pqdict
import math

import numpy as np

class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    # return self.g < other.g     
    return self.h < other.h


class AStar(object):
  step_size = 0.3
  _directions = []
  for dx in [-step_size, 0, step_size]:
    for dy in [-step_size, 0, step_size]:
      for dz in [-step_size, 0, step_size]:
        if dx == 0 and dy == 0 and dz == 0:
          continue
        _directions.append(np.array([dx, dy, dz]))

  @staticmethod
  def get_neighbors(coord):
    return [coord + d for d in AStar._directions]

  @staticmethod
  def get_heuristic(coord, goal_coord):
    return np.linalg.norm(coord - goal_coord)

  @staticmethod
  def plan(start_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = environment.Graph
    OPEN = pqdict()

    # TODO: Implement A* here
    
    # Start point info
    start_key = tuple(np.round(start_coord, 3))

    # Initialization
    if start_key not in Graph:
      # current node
      curr = AStarNode(start_key, start_coord, environment.getHeuristic(start_coord))
      curr.g = 0
      Graph[start_key] = curr
    else:
      curr = Graph[start_key]
      curr.g = 0

    # LPA*
    # Update h value and put back in OPEN
    for key, node in Graph.items():
      node.h = environment.getHeuristic(node.coord)
      node.closed = False
      if node.g != math.inf:
        OPEN.additem(node, node.g + epsilon * node.h)

    goal_reached_node = None

    # A*
    while len(OPEN) > 0:
      curr_node = OPEN.pop()
      curr_node.closed = True

      dist_to_goal = environment.getHeuristic(curr_node.coord)

      if dist_to_goal <= 0.3:
        goal_reached_node = curr_node
        break

      neighbors = AStar.get_neighbors(curr_node.coord)

      if dist_to_goal <= 1.5:
        neighbors.append(environment.current_goal)

      for next_coord in neighbors:
        if not environment.is_valid(curr_node.coord, next_coord):
          continue

        next_key = tuple(np.round(next_coord, 3))
        cost = np.linalg.norm(next_coord - curr_node.coord)
        new_g = curr_node.g + cost

        if next_key not in Graph:
          next_h = environment.getHeuristic(next_coord)
          next_node = AStarNode(next_key, next_coord, next_h)
          Graph[next_key] = next_node
        else:
          next_node = Graph[next_key]
          if next_node.closed or new_g >= next_node.g:
            continue

        next_node.g = new_g
        next_node.parent_node = curr_node
        f_value = next_node.g + epsilon * next_node.h

        if next_node in OPEN:
          OPEN[next_node] = f_value
        else:
          OPEN.additem(next_node, f_value)

    # Rewire
    if goal_reached_node is None:
      return np.array([start_coord])
    
    path = []
    curr = goal_reached_node
    while curr is not None:
      path.append(curr.coord)
      curr = curr.parent_node

    return np.array(path[::-1])



