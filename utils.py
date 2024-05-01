import numpy as np

def are_goals_adjacent(goals):
    for i in range(len(goals)):
        for j in range(i + 1, len(goals)):
            if (abs(goals[i][0] - goals[j][0]) <= 1 and
                abs(goals[i][1] - goals[j][1]) <= 1):
                return True
    return False

def move(loc, dir):
    # directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (1,1), (-1,-1), (1,-1), (-1,1)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def parse_given_files(file1path: str, file2path: str):
    starts = []
    ends = []
    with open(file1path, 'r') as file1:
        lines1 = file1.readlines()

    for line in lines1:
        values = [int(value.strip()) for value in line.split(",")]
        starts.append((values[0], values[1]))
        ends.append((values[2], values[3]))

    with open(file2path, "r") as file2:
        lines2 = file2.readlines()
    
    N = int(lines2[0].strip())
    
    map_array = np.full((N, N), False, dtype=bool)
    
    restricted_indexes = []
    
    for point in lines2[1:]:
        x, y = map(int, point.strip().split(", "))
        restricted_indexes.append((x,y))
        map_array[x][y] = True

    return map_array.tolist(), starts, ends

def write_result(paths, result_file_name):
    updated_path = []
    for path in paths:
        new_path = []
        for i in range(len(path)-1):
            
            step = path[i], path[i+1]
            new_path.append(step)
        
        updated_path.append(new_path)
    print(updated_path)

    with open(result_file_name, "w") as file:
        for robot_path in updated_path:
            for x_from, y_from, x_to, y_to in [(start[0], start[1], end[0], end[1]) for start, end in robot_path]:
                file.write(f"{x_from}, {y_from}, {x_to}, {y_to} \n")
            file.write("\n")

