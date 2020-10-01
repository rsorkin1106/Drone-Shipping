// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0
#include <getopt.h>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <numeric>
#include <string.h>
#include <cmath>

using namespace std;


enum class Loc : char
{
    medical,
    normal,
    border
};
class Drone
{

public:

    //A vertex on the graph
    struct Travel
    {
        int x = 0, y = 0;
        bool visited = false;
    };
    
    //Every variable a vertex needs to calculate Prim's
    struct Prims
    {
        double distance = numeric_limits<double>::infinity();
        Loc loc = Loc::normal;
        int x = 0, y = 0;
        bool visited = false;
        int prev = -1;

        Prims(const Travel& t)
            : x(t.x), y(t.y) {}
        Prims() {}
    };

private:
    vector<Prims> prim;
    vector<Travel> path;
    vector<size_t> optOrder, bestPath;
    double totOptDist = 0, bestOptDist;

public:
    char mode;
    
    //Parses command line arguments
    void getOptions(int argc, char** argv)
    {
        int option_index = 0, option = 0;
        bool m = false;

        struct option longOpts[] = { {"mode", required_argument, nullptr, 'm' },
                                    {"help", no_argument, nullptr, 'h'},
                                    { nullptr, 0, nullptr, '\0' } };

        while ((option = getopt_long(argc, argv, "m:h", longOpts, &option_index)) != -1) {
            switch (option) {
            case 'm':
                if (strcmp(optarg, "MST") != 0 || strcmp(optarg, "FASTTSP") != 0 || strcmp(optarg, "OPTTSP") != 0)
                    mode = optarg[0];
                else
                {
                    cerr << "Mode has wrong type";
                    exit(1);
                }
                m = true;
                break;

            case 'h':
                cout << "Command line options: -m <MODE> or -h";
                exit(0);

            default:
                exit(1);
            }
        }
        if (!m)
        {
            cerr << "need to specify mode";
            exit(1);
        }
    }

    
    //Reads in every vertex
    void getCoords()
    {
        bool normal = false, medical = false, border = false;
        int num, x, y;
        cin >> num;
        
        if (mode == 'M')
            prim.reserve(num);
        else
            path.reserve(num);

        for (int i = 0; i < num; ++i)
        {
            cin >> x >> y;
            
            //Necessary variables for MST
            if (mode == 'M')
            {
                Prims temp;
                if ((x == 0 && y < 0) || (x < 0 && y == 0) || (x == 0 && y == 0))
                {
                    border = true;
                     temp.loc = Loc::border;
                }
                else if (x < 0 && y < 0)
                {
                    medical = true;
                    temp.loc = Loc::medical;
                }
                else if (!normal && (x > 0 || y > 0))
                    normal = true;
                
                temp.x = x;
                temp.y = y;
                prim.push_back(temp);
            }
            //Necessary variables for TSP
            else
            {
                Travel temp;
                temp.x = x;
                temp.y = y;
                path.push_back(temp);
            }
        }
        if (mode == 'M')
        {
            //Error if there is a vertex in medical area and normal area, but none on the border
            if (medical && normal && !border)
            {
                cerr << "Cannot construct MST";
                exit(1);
            }
            else
                mst();
        }
        else if (mode == 'F')
            fast();
        else if (mode == 'O')
            opt();
    }

    private:
    
    //Prim's algorithm to calculate an MST
    double mst()
    {
        vector<pair<int, int>> edges;
        double totDist = 0;
        
        edges.reserve(prim.size() - 1);
        prim[0].distance = 0;
        
        for (size_t i = 0; i < prim.size(); ++i)
        {
            double minDist = numeric_limits<double>::infinity();
            size_t minDistIdx = 0;
            for (size_t j = 0; j < prim.size(); ++j)
            {
                if (!prim[j].visited && prim[j].distance < minDist)
                {
                    minDist = prim[j].distance;
                    minDistIdx = j;
                }
            }
            prim[minDistIdx].visited = true;
            totDist += prim[minDistIdx].distance;
            if (prim[minDistIdx].prev != -1)
            {
                if (int(minDistIdx) < prim[minDistIdx].prev)
                    edges.push_back({ int(minDistIdx), prim[minDistIdx].prev });
                else
                    edges.push_back({ prim[minDistIdx].prev, int(minDistIdx) });
            }
            if(i != prim.size() - 1)
                distA(int(minDistIdx));
        }

        if (mode == 'M')
        {
            cout << totDist << '\n';
            for (size_t i = 0; i < edges.size(); ++i)
            {
                cout << edges[i].first << " " << edges[i].second << '\n';
            }
        }
        return totDist;
    }
    
    //Nearest insertion of arbitrary city TSP heuristic
    double fast()
    {
        //holds the order of indices in path
        vector<int> order;
        double totDist = 0;
        double minDist = numeric_limits<double>::infinity();
        size_t minDistIdx = 0;

        order.reserve(path.size());
        path[0].visited = true;

        //Finds closest vertex
        for (size_t i = 1; i < path.size(); ++i)
        {
            double tempMin = getDist(0, int(i));
            if (tempMin < minDist)
            {
                minDist = tempMin;
                minDistIdx = i;
            }
        }

        path[minDistIdx].visited = true;
        totDist += minDist;

        order.push_back(0);
        order.push_back(int(minDistIdx));

        //Picks random city and inserts it between to connected cities
        for (size_t i = 1; i < path.size(); ++i)
        {
            if (!path[i].visited)
            {
                minDist = numeric_limits<double>::infinity();
                //Minimizes insertion between 2 edges
                for (size_t j = 0; j < order.size() - 1; ++j)
                {
                    double temp = getDist(order[j], int(i)) + getDist(int(i), order[j + 1]) - getDist(order[j], order[j + 1]);
                    if (temp < minDist)
                    {
                        minDist = temp;

                        //insert() inserts before minDistIdx
                        minDistIdx = j + 1;
                    }
                }
                //subtracts distance between {i, j} and adds distance {i, k} and {k, j}
                totDist += minDist;
                auto it = order.begin() + minDistIdx;
                order.insert(it, int(i));
                path[i].visited = true;
            }
        }

        //Adds final location distance to first
        totDist += getDist(order[0], order[order.size() - 1]);

        if (mode == 'F')
        {
            cout << totDist << '\n';
            for (size_t i = 0; i < order.size(); ++i)
            {
                cout << order[i] << " ";
            }
        }

        return totDist;
    }
    
    //Calculates optimal TSP
    void opt()
    {
        optOrder.resize(path.size());
        iota(optOrder.begin(), optOrder.end(), 0);
        
        //Uses TSP heuristic find initial shortest distance
        bestOptDist = fast();
        genPerms(1);

        cout << bestOptDist << '\n';
        for (size_t i = 0; i < bestPath.size(); ++i)
            cout << bestPath[i] << " ";
    }

    //Calculates clostest vertex by comparing distance to every other vertex
    //For MST only
    void distA(int idx)
    {
        Prims vert = prim[idx];
        double x1 = double(vert.x), y1 = double(vert.y);
        for (size_t i = 0; i < prim.size(); ++i)
        {
            if (!prim[i].visited)
            {
                if ((mode != 'M') || (!(prim[i].loc == Loc::medical && vert.loc == Loc::normal) && !(vert.loc == Loc::medical && prim[i].loc == Loc::normal)))
                {
                    double x2 = double(prim[i].x), y2 = double(prim[i].y);
                    double distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
                    if (distance < prim[i].distance)
                    {
                        prim[i].distance = distance;
                        prim[i].prev = idx;
                    }
                }
            }
        }
    }
    
    //Distance function for OPTTSP and FASTTSP
    double getDist(int idx1, int idx2)
    {
        double x1 = path[idx1].x, y1 = path[idx1].y;
        double x2 = path[idx2].x, y2 = path[idx2].y;
        return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
    }
    
    //Branch and bound of optimal TSP order
    void genPerms(size_t permLength) {
        if (permLength == optOrder.size()) {
            {
                double temp = getDist(0, int(optOrder[permLength - 1])) + totOptDist;
                if (temp < bestOptDist)
                {
                    bestOptDist = temp;
                    bestPath = optOrder;
                }
                return;
            }
        } // if
        
        if (!promising(permLength))
            return;
        for (size_t i = permLength; i < path.size(); ++i) {
            swap(optOrder[permLength], optOrder[i]);
            totOptDist += getDist(int(optOrder[permLength - 1]), int(optOrder[permLength]));
            genPerms(permLength + 1);
            totOptDist -= getDist(int(optOrder[permLength - 1]), int(optOrder[permLength]));
            swap(optOrder[permLength], optOrder[i]);
        } // for
    } // genPerms()

    void createPrim(size_t permLength)
    {
        prim.resize(optOrder.size() - permLength);
        for (size_t i = permLength; i < optOrder.size(); ++i)
        {
            prim[i - permLength] = Prims(path[optOrder[i]]);
        }
    }
    
    //Determines if distance is lower than upper bound
    bool promising(size_t permLength)
    {
        if (optOrder.size() - permLength > 6)
        {
            double dist0 = numeric_limits<double>::infinity(), distLast = numeric_limits<double>::infinity();
            createPrim(permLength);
            double tempMST = mst();

            //Finds closes unvisited vertex to first and last vertex in visited area
            for (size_t i = permLength; i < optOrder.size(); ++i)
            {
                double temp0 = getDist(int(optOrder[0]), int(optOrder[i]));
                double tempLast = getDist(int(optOrder[permLength - 1]), int(optOrder[i]));
                if (temp0 < dist0)
                    dist0 = temp0;
                if (tempLast < distLast)
                    distLast = tempLast;
            }
            return (bestOptDist >= (totOptDist + tempMST + dist0 + distLast));
        }
        return true;
    }
};
