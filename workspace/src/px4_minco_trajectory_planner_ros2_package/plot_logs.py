import numpy as np
import matplotlib.pyplot as plt
import json
import csv
import re
from matplotlib.patches import Polygon
from scipy.spatial import HalfspaceIntersection

def plot_binary_map(ax, file_path, resolution, origin):
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            # It's a list of lists.
            # Handle potential trailing commas or formatting issues if any,
            # but standard json.loads should work if it's formatted reasonably like Python lists.
            # Replace common issues to make it parseable if needed.
            # However, looking at the file it's basically JSON.
            grid = json.loads(content)
            
            grid = np.array(grid)
            
            rows, cols = grid.shape
            
            # Create a 2D meshgrid
            x = np.linspace(origin[0], origin[0] + cols * resolution, cols + 1)
            y = np.linspace(origin[1], origin[1] + rows * resolution, rows + 1)
            
            # pcolormesh or imshow
            # Note: For imshow, origin='lower' keeps y-axis going up
            # Need to transpose or check the shape if it's stored row-major (y first) or column-major (x first)
            # Usually grid[x][y] or grid[row][col] (y, x). Assuming grid[x][y] from standard ROS maps:
            # Let's plot it transposed just in case, typical for occupancy grids. 
            
            # Using pcolormesh
            # grid=1 is obstacle
            cmap = plt.cm.get_cmap('Greys') 
            ax.pcolormesh(x, y, grid.T, cmap=cmap, alpha=0.5) # obstacles are black
            
    except Exception as e:
        print(f"Error loading binary map: {e}")


def plot_sfc(ax, file_path):
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
            
        polytopes = []
        current_A = []
        current_b = []
        
        for line in lines:
            line = line.strip()
            if line.startswith("--- Polytope"):
                if current_A and current_b:
                    polytopes.append((np.array(current_A), np.array(current_b)))
                current_A = []
                current_b = []
            elif line.startswith("Constraint"):
                # Example: Constraint 0: -0 * x + -28 * y <= -5320
                match = re.match(r"Constraint \d+: ([\d\.\-]+) \* x \+ ([\d\.\-]+) \* y <= ([\d\.\-]+)", line)
                if match:
                    current_A.append([float(match.group(1)), float(match.group(2))])
                    current_b.append(float(match.group(3)))
        
        if current_A and current_b:
            polytopes.append((np.array(current_A), np.array(current_b)))
            
        # Plot each polytope
        for i, (A, b) in enumerate(polytopes):
            # To plot halfspaces, we need an interior point. 
            # A simple way is to use linear programming to find Chebychev center, 
            # or given it's a corridor, just average the bounds approx.
            # We'll use scipy HalfspaceIntersection
            try:
                # Add a bounding box to make it bounded if not already
                box_A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
                box_b = np.array([100, 100, 100, 100]) # bounds
                
                halfspaces = np.hstack((A, -b.reshape(-1, 1)))

                interior_pt = np.linalg.lstsq(A, b, rcond=None)[0] # rough guess, might not be strictly interior

                # Better interior point: Use least squares or we can just try to plot it simply
                
                # To avoid scipy issues, let's just plot the lines
                for j in range(len(b)):
                    a_vec = A[j]
                    b_val = b[j]
                    
                    if a_vec[1] != 0:
                        x_vals = np.linspace(-20, 20, 100)
                        y_vals = (b_val - a_vec[0] * x_vals) / a_vec[1]
                        ax.plot(x_vals, y_vals, 'b-', alpha=0.3)
                    else:
                        x_val = b_val / a_vec[0]
                        ax.axvline(x=x_val, color='b', alpha=0.3)
            except Exception as e:
                print(f"Error plotting polytope {i}: {e}")

    except Exception as e:
        print(f"Error loading SFC: {e}")

def plot_grid_path(ax, file_path):
    try:
        x_pts, y_pts = [], []
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader) # skip x,y
            for row in reader:
                if len(row) >= 2:
                    # These are grid indices, we need to convert to physical units!
                    # We will assume resolution=0.1s and origin=(-20,-20) for now.
                    # Will adjust.
                    x_pts.append(float(row[0]))
                    y_pts.append(float(row[1]))
        
        # We need to map grid index back to position
        # px = (idx - 200) * 0.1 .... Need the actual params.
        
        # Let's plot them raw. If they are in index space, we will just plot index space.
        
    except Exception as e:
        print(f"Error loading grid path: {e}")

def main():
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Assumed map parameters (from C++ code)
    map_res = 0.1
    map_origin = np.array([-20.0, -20.0]) # Adjust if different
    
    # Plot binary map
    plot_binary_map(ax, 'binary_map_log.txt', map_res, map_origin)
    
    # Plot SFC
    plot_sfc(ax, 'sfc_constraints.txt')
    
    # Plot grid path (Needs conversion from index to pos)
    try:
        path_x = []
        path_y = []
        with open('grid_path_log.csv', 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if len(row) >= 2:
                    idx_x = float(row[0])
                    idx_y = float(row[1])
                    px = idx_x * map_res + map_origin[0]
                    py = idx_y * map_res + map_origin[1]
                    path_x.append(px)
                    path_y.append(py)
        ax.plot(path_x, path_y, 'g.-', label='Grid Path', markersize=3)
    except Exception as e:
        pass


    # Plot gcopter trajectory
    try:
        traj_x = []
        traj_y = []
        with open('gcopter_traj.csv', 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                if len(row) >= 3:
                    traj_x.append(float(row[1]))
                    traj_y.append(float(row[2]))
        ax.plot(traj_x, traj_y, 'r-', linewidth=2, label='GCOPTER Traj')
    except Exception as e:
        pass
        
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    plt.title('Log Visualization')
    plt.savefig('visualization_output.png', dpi=300)
    print("Saved plot to visualization_output.png")

if __name__ == '__main__':
    main()
