import traci
import sumolib
import networkx as nx

# SUMO simulation command
sumo_cmd = ["sumo-gui", "-c", "osm.sumocfg", "--start", "--no-step-log"]

# Start and end edges
start_edge = "-314569224#2"
end_edge = "314567749#3"

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
        graph.add_edge(from_node, to_node, id=edge_id, base_weight=length)
    return graph

def update_weights(graph):
    """
    Update the graph weights based on real-time traffic data from simulation.
    Weights are updated as: weight = length * (1 + congestion).
    """
    for u, v, data in graph.edges(data=True):
        edge_id = data['id']
        try:
            congestion = traci.edge.getLastStepOccupancy(edge_id)
            data['weight'] = data['base_weight'] * (1 + congestion)
        except traci.TraCIException:
            data['weight'] = data['base_weight']

def print_cost_matrix(graph, start_node):
    """
    Prints the cost matrix that includes the cumulative cost to reach each node from start_node
    after running Dijkstra's algorithm.
    """
    costs = {node: float('inf') for node in graph.nodes()}
    costs[start_node] = 0

    predecessors = {node: None for node in graph.nodes()}

    unvisited_nodes = list(graph.nodes())
    
    while unvisited_nodes:
        current_node = min(unvisited_nodes, key=lambda node: costs[node])
        unvisited_nodes.remove(current_node)

        for neighbor in graph.neighbors(current_node):
            edge_weight = graph[current_node][neighbor].get('weight', graph[current_node][neighbor]['base_weight'])
            new_cost = costs[current_node] + edge_weight

            if new_cost < costs[neighbor]:
                costs[neighbor] = new_cost
                predecessors[neighbor] = current_node

    print("\nFinal Cost Matrix:")
    for node, cost in costs.items():
        print(f"Node {node}: Cost = {cost:.2f}")

    return costs, predecessors

def find_best_path(graph, start_node, end_node):
    """
    Use Dijkstra's algorithm to find the shortest path from start_node to end_node.
    """
    costs, predecessors = print_cost_matrix(graph, start_node)

    if costs[end_node] == float('inf'):
        print("No path found between the start and end nodes.")
        return None, None

    path = []
    current_node = end_node
    while current_node is not None:
        path.append(current_node)
        current_node = predecessors[current_node]

    path.reverse()
    return path, costs[end_node]

def highlight_best_path_edges(best_path, graph):
    """
    Highlight the edges along the best path by changing their color.
    """
    for u, v in zip(best_path[:-1], best_path[1:]):
        edge_id = graph[u][v]['id']
        traci.edge.setParameter(edge_id, "color", "255,0,0")  # Red color

def run_simulation():
    try:
        traci.start(sumo_cmd)
    except traci.TraCIException as e:
        print(f"TraCI failed to start: {e}")
        return

    net = sumolib.net.readNet("osm.net.xml.gz")
    graph = build_graph(net)
    update_weights(graph)

    start_node = net.getEdge(start_edge).getFromNode().getID()
    end_node = net.getEdge(end_edge).getToNode().getID()

    best_path, total_cost = find_best_path(graph, start_node, end_node)

    if best_path:
        print(f"Best path found: {best_path} with total cost: {total_cost}")
        
        # Highlight the edges along the best path
        highlight_best_path_edges(best_path, graph)

        route_id = "ambulance_route"
        traci.route.add(route_id, [graph[u][v]['id'] for u, v in zip(best_path[:-1], best_path[1:])])
        traci.vehicle.add(vehID="ambulance", routeID=route_id, typeID="ambulance", depart=0)

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

        travel_time = traci.vehicle.getTravelTime("ambulance")
        print(f"Ambulance Travel Time: {travel_time} seconds")
    else:
        print("No path found.")

    traci.close()

if __name__ == "__main__":
    run_simulation()

