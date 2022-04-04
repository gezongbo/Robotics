#ifndef TROJAN_MAP_H
#define TROJAN_MAP_H

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cfloat>
#include <math.h>
#include <fstream>
#include <sstream>
#include <climits>

// A Node is the location of one point in the map.
class Node {
  public:
    Node(){};
    Node(const Node &n){id = n.id; lat = n.lat; lon = n.lon; name = n.name; neighbors = n.neighbors; attributes = n.attributes;};
    std::string id;    // A unique id assign to each point
    double lat;        // Latitude
    double lon;        // Longitude
    std::string name;  // Name of the location. E.g. "Bank of America".
    std::vector<std::string> neighbors;  // List of the ids of all neighbor points.
    std::unordered_set<std::string> attributes;  // List of the attributes of the location.
};

class TrojanMap {
 public:
  // Constructor
  TrojanMap(){CreateGraphFromCSVFile();};
  
  // A map of ids to Nodes.
  std::unordered_map<std::string, Node> data;  

  //-----------------------------------------------------
  // Read in the data
  void CreateGraphFromCSVFile();

  //-----------------------------------------------------
  // TODO: Implement these functions and create unit tests for them:
  // Get the Latitude of a Node given its id.
  double GetLat(const std::string& id);

  // Get the Longitude of a Node given its id.
  double GetLon(const std::string& id);

  // Get the name of a Node given its id.
  std::string GetName(const std::string& id);

  // Get the id given its name.
  std::string GetID(const std::string& name);

  // Get the neighbor ids of a Node.
  std::vector<std::string> GetNeighborIDs(const std::string& id);

  // Returns a vector of names given a partial name.
  std::vector<std::string> Autocomplete(std::string name);

  // Returns lat and lon of the given the name.
  std::pair<double, double> GetPosition(std::string name);

  // Calculate location names' edit distance
  int CalculateEditDistance(std::string, std::string);

  // Find the closest name
  std::string FindClosestName(std::string name);  

  // Get the distance between 2 nodes.
  double CalculateDistance(const std::string &a, const std::string &b);

  // Calculates the total path length for the locations inside the vector.
  double CalculatePathLength(const std::vector<std::string> &path);

  // Given the name of two locations, it should return the **ids** of the nodes
  // on the shortest path.
  std::vector<std::string> CalculateShortestPath_Dijkstra(std::string location1_name,
                                                 std::string location2_name);
  std::vector<std::string> CalculateShortestPath_Bellman_Ford(std::string location1_name,
                                                 std::string location2_name);

  // Given CSV filename, it read and parse locations data from CSV file,
  // and return locations vector for topological sort problem.
  std::vector<std::string> ReadLocationsFromCSVFile(std::string locations_filename);
  
  // Given CSV filenames, it read and parse dependencise data from CSV file,
  // and return dependencies vector for topological sort problem.
  std::vector<std::vector<std::string>> ReadDependenciesFromCSVFile(std::string dependencies_filename);

  // Given a vector of location names, it should return a sorting of nodes
  // that satisfies the given dependencies.
  std::vector<std::string> DeliveringTrojan(std::vector<std::string> &location_names,
                                            std::vector<std::vector<std::string>> &dependencies);

  // Given a vector of location ids, it should reorder them such that the path
  // that covers all these points has the minimum length.
  // The return value is a pair where the first member is the total_path,
  // and the second member is the reordered vector of points.
  // (Notice that we don't find the optimal answer. You can return an estimated
  // path.)
  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_Brute_force(
      std::vector<std::string> location_ids);
  
  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_Backtracking(
      std::vector<std::string> location_ids);
  
  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_2opt(
      std::vector<std::string> location_ids);

  // Check whether the id is in square or not
  bool inSquare(std::string id, std::vector<double> &square);

  // Get the subgraph based on the input
  std::vector<std::string> GetSubgraph(std::vector<double> &square);
  
  // Given a subgraph specified by a square-shape area, determine whether there is a
  // cycle or not in this subgraph.
  bool CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square);

  // Given a location id and k, find the k closest points on the map
  std::vector<std::string> FindNearby(std::string, std::string, double, int);
  
  //----------------------------------------------------- User-defined functions

};

#endif
