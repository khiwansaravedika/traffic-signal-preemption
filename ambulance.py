import traci
import sumolib
import networkx as nx
import random
import threading
from queue import Queue

# SUMO simulation command
sumo_cmd = ["sumo-gui", "-c", "osm.sumocfg", "--start", "--no-step-log"]

# Start and end edges
start_edge = "-314569224#2"
end_edge = "314567749#3"

def random_road_condition():
    """
    Assign a random road condition value.
    """
    return random.choice(["bad", "moderate", "good"])

def build_graph(net):
    """
    Build a graph representation of the SUMO network.
    Each edge's weight is initially set to its length.
    """
    graph = nx.DiGraph()
    for edge in net.getEdges():
        edge_id = edge.getID()
        from_node = edge.getFromNode().getID()
        to_node = edge.getToNode().getID()
        length = edge.getLength()
        road_condition = random_road_condition()  # Randomly assign road condition
        condition_factor = {"bad": 2.0, "moderate": 1.5, "good": 1.0}.get(road_condition, 1.0)  # Numerical weights
        weight = length * condition_factor
        graph.add_edge(from_node, to_node, id=edge_id, base_weight=weight, road_condition=road_condition)
    return graph

def update_weights(graph):
    """
    Update the graph weights based on real-time traffic data from simulation.
    Weights are updated as: weight = length * (1 + congestion) * condition_factor.
    """
    for u, v, data in graph.edges(data=True):
        edge_id = data['id']
        try:
            congestion = traci.edge.getLastStepOccupancy(edge_id)
            road_condition = data['road_condition']
            condition_factor = {"bad": 2.0, "moderate": 1.5, "good": 1.0}.get(road_condition, 1.0)  # Numerical weights
            data['weight'] = data['base_weight'] * (1 + congestion) * condition_factor
        except traci.TraCIException:
            data['weight'] = data['base_weight']

def all_paths_bfs(graph, start_node, end_node, max_paths):
    """
    Find all possible paths from start_node to end_node using BFS.
    """
    paths = []
    queue = Queue()
    queue.put([start_node])

    while not queue.empty() and len(paths) < max_paths:
        path = queue.get()
        node = path[-1]

        if node == end_node:
            paths.append(path)
        else:
            for neighbor in graph.neighbors(node):
                new_path = list(path)
                new_path.append(neighbor)
                queue.put(new_path)

    return paths

def compute_path_cost(graph, path):
    """
    Compute the total cost of a given path.
    The cost is a combination of edge weights and the number of junctions.
    """
    edge_cost = 0
    junction_cost = len(path) - 1  # Number of junctions is one less than the number of nodes in the path

    for u, v in zip(path[:-1], path[1:]):
        edge_weight = graph[u][v].get('weight', graph[u][v]['base_weight'])
        edge_cost += edge_weight

    total_cost = edge_cost + junction_cost
    return total_cost

def find_and_process_paths(graph, start_node, end_node, max_paths, results_queue):
    """
    Find all paths using BFS and compute their costs.
    """
    paths = all_paths_bfs(graph, start_node, end_node, max_paths)
    paths_with_costs = [(path, compute_path_cost(graph, path)) for path in paths]
    results_queue.put(paths_with_costs)

def reroute_vehicle(graph, vehID, end_node):
    """
    Dynamically reroute the vehicle based on real-time conditions.
    """
    current_edge = traci.vehicle.getRoadID(vehID)
    if current_edge in ("", "waiting"):
        return
    
    current_edge = current_edge.split("#")[0]
    
    if current_edge in graph.nodes:
        current_node = current_edge
        rerouted_path = None
        min_cost = float('inf')

        for neighbor in graph.neighbors(current_node):
            path, cost = dijkstra(graph, neighbor, end_node)
            if cost < min_cost:
                min_cost = cost
                rerouted_path = path

        if rerouted_path:
            route_id = f"reroute_{vehID}"
            traci.route.add(route_id, rerouted_path)
            traci.vehicle.setRouteID(vehID, route_id)
            print(f"Vehicle {vehID} rerouted to: {rerouted_path} with cost: {min_cost}")

def run_simulation():
    try:
        print("Starting SUMO simulation...")  # Debugging: Start message
        traci.start(sumo_cmd)
    except traci.TraCIException as e:
        print(f"TraCI failed to start: {e}")
        return

    print("Building graph...")  # Debugging: Building graph message
    net = sumolib.net.readNet("osm.net.xml.gz")
    graph = build_graph(net)
    update_weights(graph)

    start_node = net.getEdge(start_edge).getFromNode().getID()
    end_node = net.getEdge(end_edge).getToNode().getID()

    results_queue = Queue()
    thread = threading.Thread(target=find_and_process_paths, args=(graph, start_node, end_node, 5, results_queue))
    thread.start()
    thread.join()

    best_paths = results_queue.get()

    if best_paths:
        valid_paths = []
        for i, (path, cost) in enumerate(best_paths):
            try:
                road_conditions = [graph[u][v]['road_condition'] for u, v in zip(path[:-1], path[1:])]
                number_of_junctions = len(path) - 1  # Number of junctions
                valid_paths.append((path, cost, number_of_junctions, road_conditions))
            except KeyError as e:
                print(f"Error accessing path {i+1}: {e}")
                continue

        for i, (path, cost, number_of_junctions, road_conditions) in enumerate(valid_paths):
            print(f"Path {i+1}:")
            print(f"  Nodes: {path}")
            print(f"  Total Cost: {cost:.2f}")
            print(f"  Number of Junctions: {number_of_junctions}")
            print(f"  Road Conditions: {road_conditions}")

        best_path, best_cost, _, _ = min(valid_paths, key=lambda x: x[1])
        print(f"\nBest Path:")
        print(f"  Nodes: {best_path}")
        print(f"  Total Cost: {best_cost:.2f}")

        route_id = "ambulance_route"
        traci.route.add(route_id, [graph[u][v]['id'] for u, v in zip(best_path[:-1], best_path[1:])])
        traci.vehicle.add(vehID="ambulance", routeID=route_id, typeID="ambulance", depart=5)  # Depart at 5 sec

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            reroute_vehicle(graph, "ambulance", end_node)

        travel_time = traci.vehicle.getTravelTime("ambulance")
        print(f"Ambulance Travel Time: {travel_time} seconds")
    else:
        print("No paths found.")

    traci.close()

if __name__ == "__main__":
    run_simulation()
