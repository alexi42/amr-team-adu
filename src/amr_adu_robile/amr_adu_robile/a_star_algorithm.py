import heapq
import threading
from .conversion_script import convert_world_coordinates_to_grid

"""
Adapted code for A*-algorithm found on https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/
"""


class GridCell():
    """
    Class implements A* algorithm
    """
    def __init__(self, ROW, COL, RESOLUTION):
        # row index
        self.parent_i = 0
        # column index
        self.parent_j = 0
        # cost of the cell g+h
        self.f = float('inf')
        # cost from start to this cell
        self.g = float('inf')
        # cost from this cell to destination
        self.h = 0

        # Occupancy grid parameters
        self.ROW = ROW
        self.COL = COL
        self.RESOLUTION = RESOLUTION

        # Lock for threading
        self.rlock = threading.RLock()

    # check if the provided cell is valid
    def is_valid(self, row, col):
        if row is None or col is None:
            return False
        return (row >= 0) and (row < self.ROW) and (col >= 0) and (col < self.COL)

    # check if the given cell is free, 1 = occupied, 0 = free
    def is_available(self, grid, row, col):
        return grid[row][col] == 0

    # check if the provided cell is the destination
    def is_destination(self, row, col, dest):
        return row == dest[0] and col == dest[1]

    # calculate heuristic value as euclidean distance
    def calculate_h_value(self, row, col, dest):
        return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

    # trace the path from the start to the destination
    def trace_path(self, cell_details, dest):
        path = []
        row = dest[0]
        col = dest[1]

        # Trace the path from destination to source using parent cells
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        path.append((row, col))
        # Reverse the path to get the path from source to destination
        path.reverse()
        return path

    def a_star_search(self, grid, src, dest, start):
        # check if source and destination are valid
        if start is None:
            print("START not initialized yet. Waiting for odometry...")
            return

        src_grid = convert_world_coordinates_to_grid(
            src, start=start, res=self.RESOLUTION
        )
        dest_grid = convert_world_coordinates_to_grid(
            dest, start=start, res=self.RESOLUTION
        )

        print(f"World grid origin: {start}")
        print(f"Source world: {src} -> grid: {src_grid}")
        print(f"Goal world: {dest} -> grid: {dest_grid}")

        if not self.is_valid(src_grid[0], src_grid[1]) or not self.is_valid(dest_grid[0], dest_grid[1]):
            print("Source or destination is invalid.")
            return

        # check if we are already at destination
        if self.is_destination(src_grid[0], src_grid[1], dest_grid):
            print("We are lready at destination.")
            return

        # Initilize the visited cells
        closed_list = [[False for _ in range(self.COL)] for _ in range(self.ROW)]
        # Initialize the details of each cell
        cell_details = [[GridCell(ROW=self.ROW, COL=self.COL, RESOLUTION=self.RESOLUTION) for _ in range(self.COL)] for _ in range(self.ROW)]

        # Initialize the start cell details
        i = src_grid[0]
        j = src_grid[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialize the open list (cells to be visited) with the start cell
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))

        # Initialize the flag for whether destination is found
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closed_list[i][j] = True

            # For each direction, check the successors
            directions = [
                (0, 1), (0, -1), (1, 0), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)
            ]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]
                # If the successor is valid, unblocked, and not visited
                if self.is_valid(new_i, new_j) and self.is_available(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                    # If the successor is the destination
                    if self.is_destination(new_i, new_j, dest_grid):
                        # Set the parent of the destination cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        found_dest = True
                        return self.trace_path(cell_details, dest_grid)
                    else:
                        # Calculate the new f, g, and h values
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j, dest_grid)
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        # If the destination is not found after visiting all cells
        if not found_dest:
            print("Failed to find the destination cell")
