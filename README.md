# Drone-Shipping

A program that uses Prim's algorithm as well as TSP heuristics and branch and bound to find the optimal path for a drone delivering orders

## Usage

Using "make" from the makefile will compile puzzle

$./drone [--mode MST/FASTTSP/OPTTSP] [--help]

--mode - A required flag with 3 possible arguments:
  MST - Takes input map and finds an mst for the graph based on certain rules (See below)
  FASTTSP - Takes input map and uses TSP heuristics
  OPTTSP - Uses branch and bound to determine the most optimal TSP path
  
--help - Prints possible command line arguments

## Objective

Map Input: The first line must contain a single number denoting the number of locations on campus.
That will be followed by a list of integer x/y, coordinates in the form: x y.

MST - Given a coordinate grid-like map, any trips to and from points in the 3rd quadrant bounded by
the negative x-axis and negative y-axis (point (0,0) is included) can only be made if stopped at a vertex on the border.
When this flag is chosen, following the above rules, the program will calculate an MST to reach every point and will output
the weight of the MST followed by 2 nodes per line to indicate the edges in the tree.


FASTTSP - Given a coordinate grid-like map, calculate a good (but not necessarily optimal) TSP route between all the points.
Uses the nearest insertion of arbitrary city to do so. Prints the total distance traveled followed by the vertices in the order they
were visited.


OPTTSP - Given a coordinate grid-like map, calculate the most optimal TSP route between all the poins. Uses branch and bound with the
earlier-constructed MST as a lower bound to determine the shortest distance as quickly as possible.


## Example Input

(sample-d.txt)

30

50 86

26 86

80 78

93 24

50 27

17 9

19 92

17 65

2 51

76 5

40 52

35 50

52 69

1 77

84 64

59 77

41 55

27 9

93 6

17 40

42 91

18 30

5 16

86 1

97 56

53 27

82 80

67 58

50 47

89 82


## Sample Output

(sample-d-MST)

30

50 86

26 86

80 78

93 24

50 27

17 9

19 92

17 65

2 51

76 5

40 52

35 50

52 69

1 77

84 64

59 77

41 55

27 9

93 6

17 40

42 91

18 30

5 16

86 1

97 56

53 27

82 80

67 58

50 47

89 82
