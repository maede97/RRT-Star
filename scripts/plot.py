import numpy as np
import matplotlib.pyplot as plt
import json

def plot():
    # Load data
    json_data = json.loads(open('../build/data.json','r').read())

    # Draw Edges
    data_edges = np.array(json_data['edges'])
    plt.plot([data_edges[:, 0], data_edges[:, 2]], [data_edges[:, 1], data_edges[:, 3]], color="k")

    # Draw Vertices
    data_vertices = np.array(json_data['vertices'])
    plt.scatter(data_vertices[:,0], data_vertices[:,1])

    # Draw Obstacles
    data_obstacles = np.array(json_data['obstacles'])
    num_obstacles = data_obstacles.shape[0]
    for row in range(num_obstacles):
        c = plt.Circle((data_obstacles[row, 0], data_obstacles[row, 1]), data_obstacles[row, 2],color="blue",alpha=0.6)
        plt.gca().add_patch(c)

    # Draw Path
    data_path = np.array(json_data['path'])
    plt.plot([data_path[:, 0], data_path[:, 2]], [data_path[:, 1], data_path[:, 3]], color="r")

    plt.gca().set_aspect('equal')

    # Compute nicer plot size
    min_x = np.min(data_vertices[:,0])
    max_x = np.max(data_vertices[:,0])
    min_y = np.min(data_vertices[:,1])
    max_y = np.max(data_vertices[:,1])
    diff_x = max_x - min_x
    diff_y = max_y - min_y
    add_x = 0.05 * diff_x
    add_y = 0.05 * diff_y
    plt.xlim([min_x - add_x, max_x + add_x])
    plt.ylim([min_y - add_y, max_y + add_y])

    # Remove ticks
    plt.xticks([],[])
    plt.yticks([],[])
    plt.tight_layout()
    
    plt.show()
    #plt.savefig("../images/example.png",bbox_inches="tight")

if __name__ == '__main__':
    plot()
