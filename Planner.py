import numpy as np
from aabbtree import AABB, AABBTree

from astar import AStar

class MyPlanner:
  __slots__ = ['boundary', 'blocks', 'tree', 'Graph', 'current_goal']
  # __slots__ = ['boundary', 'blocks', 'tree']
  # __slots__ = ['boundary', 'blocks']
  
  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks
    self.tree = AABBTree()

    self.Graph = {} 
    self.current_goal = None

    self._build_aabb_tree()

  def _build_aabb_tree(self):
    for i, block in enumerate(self.blocks):
      # block: [xmin, ymin, zmin, xmax, ymax, zmax]
      # aabbtree: [(xmin, xmax), (ymin, ymax), (zmin, zmax)]
      limits = [
        (block[0], block[3]),
        (block[1], block[4]),
        (block[2], block[5])
      ]
      aabb = AABB(limits)
      self.tree.add(aabb, i)
  
  def getHeuristic(self, coord):
    return np.linalg.norm(np.array(coord) - self.current_goal)

  def is_valid(self, p1, p2):
    margin = 1e-4
    b = self.boundary[0]

    if not (b[0] <= p2[0] <= b[3] and 
            b[1] <= p2[1] <= b[4] and 
            b[2] <= p2[2] <= b[5]):
        return False
    
    if self.blocks.shape[0] == 0:
        return True
    
    # Broad-Phase: Build AABB and lookup Tree
    min_pt = np.minimum(p1, p2) - margin
    max_pt = np.maximum(p1, p2) + margin

    segment_limits = [
      (min_pt[0], max_pt[0]),
      (min_pt[1], max_pt[1]),
      (min_pt[2], max_pt[2])
    ]

    segment_aabb = AABB(segment_limits)
    candidate_indices = list(self.tree.overlap_values(segment_aabb))

    if len(candidate_indices) == 0:
      return True

    # Narrow-Phase: Slab Method applied only for candidiates
    candidates = np.atleast_2d(self.blocks[candidate_indices, :6])
    candidates_min = candidates[:, :3] - margin
    candidates_max = candidates[:, 3:6] + margin

    d = p2 - p1
    d_safe = np.where(d == 0, 1e-8, d)

    t1 = (candidates_min - p1) / d_safe
    t2 = (candidates_max - p1) / d_safe

    t_min_dim = np.minimum(t1, t2)
    t_max_dim = np.maximum(t1, t2)

    t_enter = np.max(t_min_dim, axis=1)
    t_exit = np.min(t_max_dim, axis=1)

    collision_mask = (t_enter <= t_exit) & (t_enter <= 1.0) & (t_exit >= 0.0)

    if np.any(collision_mask):
      return False

    return True

  def check_collision(self, p1, p2):
    return self.is_valid(p1, p2)

  def plan(self, start, goal):
    self.current_goal = np.array(goal)
    path = AStar.plan(start, self, epsilon=5.0)
    return path
