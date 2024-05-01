from queue import PriorityQueue
from utils import move
from low_level_planner import a_star
import random
import time

def get_heuristics(my_map, goal):
    """
    Calculate heuristic values from each point on the map to the goal using Dijkstra's algorithm.
    """
    open_list = PriorityQueue()
    closed_list = dict()

    root = {'loc': goal, 'cost': 0}
    open_list.put((root['cost'], goal, root))
    closed_list[goal] = root

    print(f"Calculating heuristics for goal at {goal}")
    
    while not open_list.empty():
        cost, loc, curr = open_list.get()
        
        for dir in range(8):
            child_loc = move(loc, dir)
            
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
               continue

            child_cost = cost + 1
            child = {'loc': child_loc, 'cost': child_cost}
            
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    open_list.put((child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                open_list.put((child_cost, child_loc, child))

    h_values = {loc: node['cost'] for loc, node in closed_list.items()}
    return h_values

def total_cost(paths):
    """
    Calculate the total cost of a list of paths, where cost is defined as the total number of moves.

    """
    cost = sum(len(path) - 1 for path in paths)
    print(f"Total cost of paths: {cost}")
    return cost


def get_collision_details(pathA, pathB):
    """
    Determine if there are collisions between two paths at any timestep.

    """
    
    if len(pathA)<len(pathB):
        path1 = pathA
        path2 = pathB

    elif len(pathB)<=len(pathA):
        path1 = pathB
        path2 = pathA

    long_length = len(path2)

    for t in range(long_length):
        if t < len(path1):
            pos1 = get_location(path1, t)

        else:
            pos1 = path1[-1]

        pos2 = get_location(path2, t)

        if pos1 == pos2:
            return [pos1], t, 'vertex'
        
        if t < long_length - 1:
            if (t+1) < len(path1):
                next_pos1 = get_location(path1, t+1)

            else:
                next_pos1 = path1[-1]

            next_pos2 = get_location(path2, t+1)

            if pos1 == next_pos2 and pos2 == next_pos1:
                return [pos1, next_pos1], t+1, 'edge'
            
        
def get_location(path, time):
    """
    This functions gets the location of the king at a certain time step in his path.
    """
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1] 
    
def detect_collisions(paths):
    """
    Detect all collisions among a set of paths.

    """
    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            coll_data = get_collision_details(paths[i], paths[j])
            if coll_data:
                collisions.append({
                    'k1': i,
                    'k2': j,
                    'loc': coll_data[0],
                    'timestep': coll_data[1],
                    'type': coll_data[2]
                })
    print(f"Detected collisions: {collisions}")
    return collisions

def get_constraints_from_collision(collision: dict, paths, goals: list):

    """
    This functions takes up a collision (dictionary) data as an input and splits it into two constraints, one for each king, based on type of collision 
    """
    constraints = []

    # Handles 'vertex' type collisions
    if collision['type'] == 'vertex':
        constraints.append({
            'king': collision['k1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'king': collision['k2'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })

    # Handles 'edge' type collisions
    elif collision['type'] == 'edge':
        constraints.append({
            'king': collision['k1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'king': collision['k2'],
            'loc': list(reversed(collision['loc'])), # Reverse the edge for k2
            'timestep': collision['timestep'],
            'final': False
        })

    return constraints

def multi_agent_path_planner(my_map, starts, goals):
    """
    This function tries to find paths for multiple agents (kings) from their start positions to their respective goal positions on a grid map. It uses a combination of A* search for individual
    path planning and a high-level search to manage conflicts between paths of different agents.
    """
    start_time = time.time()
    max_duration = 600  # 10 minutes in seconds

    num_of_kings = len(goals)
    num_of_generated = 0
    num_of_expanded = 0

    open_list = PriorityQueue()
    heuristics = [get_heuristics(my_map, goal) for goal in goals]

    root = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}

    # Initialize paths using A* for each agent based on individual goals and no constraints
    for i in range(num_of_kings):
        path = a_star(my_map, starts[i], goals[i], heuristics[i], i, root['constraints'])
        if path is None:
            raise Exception('No solutions found for initial paths')
        root['paths'].append(path)

    root['cost'] = total_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])
    open_list.put((len(root['collisions']), root['cost'], num_of_generated, root))
    num_of_generated += 1

    best_solution = None

    print("Starting high-level search...")

    # Main loop for the high-level search
    while not open_list.empty():
        current_time = time.time()
        if current_time - start_time > max_duration:
            print("Time limit reached. Terminating with best found solution.")
            return best_solution if best_solution is not None else None

        _, _, _, P = open_list.get()
        num_of_expanded += 1

        # If no collisions, a valid solution is found
        if not P['collisions']:
            print("Solution found without collisions.")
            return P['paths']

        # Update the best solution if current is better
        if best_solution is None or P['cost'] < total_cost(best_solution):
            best_solution = P['paths']

        # Handle collisions and generate new nodes with updated constraints
        collision = random.choice(P['collisions'])
        constraints = get_constraints_from_collision(collision, P['paths'], goals)

        for constraint in constraints:
            Q = {'cost': 0, 'constraints': [*P['constraints'], constraint], 'paths': P['paths'].copy(), 'collisions': []}
            king = constraint['king']

            # Recompute the path for the agent with the new constraint
            path = a_star(my_map, starts[king], goals[king], heuristics[king], king, Q['constraints'])
            if path:
                Q['paths'][king] = path
                Q['collisions'] = detect_collisions(Q['paths'])
                Q['cost'] = total_cost(Q['paths'])
                open_list.put((len(Q['collisions']), Q['cost'], num_of_generated, Q))
                num_of_generated += 1
                print(f"Generated new node with path cost: {Q['cost']} and collisions: {len(Q['collisions'])}")

    print("No solutions found after full exploration.")
    return best_solution
