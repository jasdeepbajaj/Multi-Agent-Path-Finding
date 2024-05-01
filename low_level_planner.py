from utils import move
from queue import PriorityQueue

def build_constraint_table(constraints, king):
    """
    Builds a dictionary where the keys are timesteps and the values are lists of constraints for that timestep.
    """
    c_table = dict()
    for c in constraints:
        # if (not 'positive' in c.keys()):
        #     c['positive'] = False
        if c['king'] == king:
            timestep = c['timestep']
            if timestep not in c_table:
                c_table[timestep] = [c]
            else:
                c_table[timestep].append(c)
    return c_table


def flatten_constraints(list_of_constraints):
    """
    Flattens a list of constraint lists into a single list of constraints.
    """
    constraints = []
    for constr_list in list_of_constraints:
        for c in constr_list:
            constraints.append(c)
    return constraints

def get_path(goal_node):
   """
   Reconstructs the path from the start node to the goal node using the parent pointers.
   """
   path = []
   curr = goal_node
   while curr is not None:
       path.append(curr['loc'])
       curr = curr['parent']
   path.reverse()
   return path

def is_goal_constrained(goal_loc, timestep, constraint_table):
    """
    Checks if the given goal location is constrained by any constraints at a given timestep.

    """
    constraints = [c for t, c in constraint_table.items() if t > timestep]
    constraints = flatten_constraints(constraints)
    for c in constraints:
        if [goal_loc] == c['loc']:
            return True
    return False

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
   """
   Checks if the next location and time are constrained based on the given constraint table.
   """
   # Check if there are constraints for the next_time
   if next_time in constraint_table:
       constraints = constraint_table[next_time]
       
       # Check if there is a vertex constraint at next_loc or edge constraint when moved from curr_loc to next_loc at that time
       for c in constraints:
           if [next_loc] == c['loc'] or [curr_loc, next_loc] == c['loc']:
               return True

   # Check constraints for previous time steps
   else:
       constraints = [c for t, c in constraint_table.items() if t < next_time]
       constraints = flatten_constraints(constraints)
       # Check if the next_loc is constrained and the constraint is final (indefinite)
       for c in constraints:
           if [next_loc] == c['loc'] and c['final']:
               return True

   return False

def compare_nodes(n1, n2):
    """
    Compares two nodes based on their total cost (g_val + h_val).
    """
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

  

def a_star(my_map, start_loc, goal_loc, h_values, king, constraints):
    """
    This function implements the A* algorithm in space-time domain to find shortest path from start to goal with given constraints that is depenedent on both and space and time
    """
    open_list = PriorityQueue()
    closed_list = dict()
    
    # Initialize the root node with the start location
    h_value = h_values[start_loc]
    c_table = build_constraint_table(constraints, king)
    root = {'loc': start_loc, 
            'g_val': 0, 
            'h_val': h_value, 
            'parent': None, 
            'time': 0}
    
    open_list.put((root['g_val'] + root['h_val'], root['h_val'], root['loc'], root)) # Priority queue to store nodes to be explored
    closed_list[(start_loc, 0)] = root # Dictionary to store explored nodes
    
    max_map_width = max([len(e) for e in my_map])
    while not open_list.empty():
        _,_,_,curr = open_list.get() #Get the node with lowest cost from unexplored nodes
        
        # Check if the current node is the goal and not constrained
        if curr['loc'] == goal_loc and not is_goal_constrained(goal_loc, curr['time'], c_table):
            return get_path(curr)
        

        for direction in range(5): #5 
            if direction < 4: #4 #explore all directions
                child_loc = move(curr['loc'], direction)

                #check if child location is out of bounds or an obstacle
                if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= max_map_width or my_map[child_loc[0]][child_loc[1]]:
                    continue
                
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'parent': curr,
                         'time': curr['time'] + 1}

            
            else: #wait at the same location 
                child = {'loc': curr['loc'],
                         'g_val': curr['g_val'] + 1,  
                         'h_val': curr['h_val'],
                         'parent': curr,
                         'time': curr['time'] + 1}
                
            # Check if the child node is constrained at that time_step
            if is_constrained(curr['loc'], child['loc'], child['time'], c_table):
                continue

            if (child['loc'], child['time']) in closed_list: # Check if the child node has been explored before
                existing_node = closed_list[(child['loc'], child['time'])]
                if compare_nodes(child, existing_node): #check if the child node is better than the existing node
                    closed_list[(child['loc'], child['time'])] = child
                    open_list.put((child['g_val'] + child['h_val'], child['h_val'], child['loc'], child))
            else:
                 # Add the child node to the closed list and open list
                closed_list[(child['loc'], child['time'])] = child
                open_list.put((child['g_val'] + child['h_val'], child['h_val'], child['loc'], child))

    return None # No path found