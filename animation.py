from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

def animation(my_map, starts, goals, paths, result_file_name):

    Colors = ['green', 'blue', 'orange']
    patches = []
    T = 0
    agents = dict()
    agent_names = dict()
    artists = []

    mymap = np.flip(np.transpose(my_map), 1)

    starts_new = []
    for start in starts:
        starts_new.append((start[1], len(mymap[0]) - 1 - start[0]))

    goals_new = []
    for goal in goals:
        goals_new.append((goal[1], len(mymap[0]) - 1 - goal[0]))

    paths_new = []
    if paths:
        for path in paths:
            paths_new.append([])
            for loc in path:
                paths_new[-1].append((loc[1], len(mymap[0]) - 1 - loc[0]))

    fig = plt.figure(frameon=False, figsize=(10, 10))
    ax = fig.add_subplot(111, aspect='equal')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

    x_min = -0.5
    y_min = -0.5
    x_max = len(mymap) - 0.5
    y_max = len(mymap[0]) - 0.5
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.grid(True)

    x_ticks = np.arange(0.5, len(my_map[1]) + 0.5, 1)
    y_ticks = np.arange(0.5, len(my_map) + 0.5, 1)
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)

    patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
    for i in range(len(mymap)):
        for j in range(len(mymap[0])):
            if mymap[i][j]:
                patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))


    for i, goal in enumerate(goals_new):
        patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],edgecolor='black', alpha=0.5))

    for i in range(len(paths_new)):
        name = str(i)
        agents[i] = Circle((starts_new[i][0], starts_new[i][1]), 0.3, facecolor=Colors[i % len(Colors)], edgecolor='black')
        agents[i].original_face_color = Colors[i % len(Colors)]
        patches.append(agents[i])
        T = max(T, len(paths_new[i]) - 1)
        agent_names[i] = ax.text(starts_new[i][0], starts_new[i][1] + 0.25, name)
        agent_names[i].set_horizontalalignment('center')
        agent_names[i].set_verticalalignment('center')
        artists.append(agent_names[i])


    def init_func():
        for p in patches:
            ax.add_patch(p)
        for a in artists:
            ax.add_artist(a)
        return patches + artists

    def get_state(t, path):
            if int(t) <= 0:
                return np.array(path[0])
            elif int(t) >= len(path):
                return np.array(path[-1])
            else:
                pos_last = np.array(path[int(t) - 1])
                pos_next = np.array(path[int(t)])
                pos = (pos_next - pos_last) * (t - int(t)) + pos_last
                return pos

    def animate_func(t):
        for k in range(len(paths_new)):
            pos = get_state(t / 10, paths_new[k])
            agents[k].center = (pos[0], pos[1])
            agent_names[k].set_position((pos[0], pos[1] + 0.5))

        # reset all colors
        for _, agent in agents.items():
            agent.set_facecolor(agent.original_face_color)


        return patches + artists

    anima = FuncAnimation(fig, animate_func, init_func=init_func, frames=int(T + 1) * 10, interval=100, blit=True)
    # writer = FFMpegWriter(fps=10, metadata=dict(artist='Me'), bitrate=1800)

    anima.save(result_file_name, writer='pillow')
    plt.show()