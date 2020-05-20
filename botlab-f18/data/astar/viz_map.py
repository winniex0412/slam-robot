import csv
import matplotlib.pyplot as plt # only needed for plotting
import sys

def importMap(FILENAME):
    """ Expect file to be formated like:
      // 0 0 0 0 
    """
    print "Reading map file... "
    grid = []
    with open(FILENAME, 'rb') as mapfile:
        csv_reader = csv.reader(mapfile, delimiter=' ')
        next(csv_reader, None) # skip header row
        for row in csv_reader:
            next_grid_row = []
            for i, val in enumerate(row):
                val = val.strip()
                print 'val: ', val, "*"
                if val != row[-1]:
                    next_grid_row.append(float(val))
            grid.append(next_grid_row)

    print grid
    return grid

def plotOGrid(grid):
    x = []
    y = []
    for nrow in range(len(grid)):
        for ncol in range(nrow):
            if grid[nrow][ncol]>0:
                x.append(nrow)
                y.append(ncol)

    print "Number of obstacles: ", len(x)
    print "Rows in grid: ", len(grid)
    print "Cols in grid: ", len(grid[0])
    plt.scatter(x, y)
    plt.show()
    return 



def main():
    grid = importMap("maze.map")
    plotOGrid(grid)
    
if __name__ == "__main__":
   # stuff only to run when not called via 'import' here
   main()