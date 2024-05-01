from utils import parse_given_files, write_result, are_goals_adjacent
from main_planner import multi_agent_path_planner
from animation import animation

if __name__ == "__main__":

    file1path = "tests/4/kings.txt"
    file2path = "tests/4/map.txt"
    my_map, starts, goals = parse_given_files(file1path, file2path)

    goal_adjacent = are_goals_adjacent(goals)

    if not goal_adjacent:
        paths = multi_agent_path_planner(my_map, starts, goals) #computes collision free paths for multi-robot system
        print(paths)
        write_result(paths, result_file_name="solution.txt") #writes the result in a text file
        animation(my_map, starts, goals, paths, result_file_name = "solution.gif") #outputs an animation of the robots moving

    else:
        print("NO Solution, since some goals are adjacent to one another")

    

    
    