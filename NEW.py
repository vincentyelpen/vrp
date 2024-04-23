from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():

    data = {}
    data['distance_matrix'] = [
        [0, 16, 15, 13, 15, 38, 9, 29, 20, 28, 11, 8, 25, 4, 22],
        [16, 0, 15, 33, 9, 14, 35, 26, 26, 57, 11, 24, 66, 13, 35],
        [15, 15, 0, 43, 10, 27, 34, 15, 10, 46, 22, 20, 67, 16, 45],
        [13, 33, 43, 0, 37, 26, 7, 54, 53, 16, 25, 10, 28, 14, 9],
        [15, 9, 10, 37, 0, 21, 40, 21, 20, 40, 16, 28, 61, 17, 39],
        [38, 14, 27, 26, 21, 0, 28, 56, 67, 37, 9, 20, 52, 15, 28],
        [9, 35, 34, 7, 40, 28, 0, 45, 31, 24, 11, 4, 21, 8, 15],
        [29, 26, 15, 54, 21, 56, 45, 0, 13, 89, 33, 33, 102, 29, 56],
        [20, 26, 10, 53, 20, 67, 31, 13, 0, 99, 32, 29, 99, 25, 56],
        [28, 57, 46, 16, 40, 37, 24, 89, 99, 0, 29, 22, 31, 26, 7],
        [11, 11, 22, 25, 16, 9, 11, 33, 32, 29, 0, 8, 49, 7, 28],
        [8, 24, 20, 10, 28, 20, 4, 33, 29, 22, 8, 0, 34, 5, 18],
        [25, 66, 67, 28, 61, 52, 21, 102, 99, 31, 49, 34, 0, 28, 31],
        [4, 13, 16, 14, 17, 15, 8, 29, 25, 26, 7, 5, 28, 0, 14],
        [22, 35, 45, 9, 39, 28, 15, 56, 56, 7, 28, 18, 31, 14, 0],
    ]
    data['demands'] = [0, 389, 370, 359, 341, 327, 326, 290, 265, 262, 261, 261, 248, 220, 220]
    data['vehicle_capacities'] = [2198, 2198]
    data['num_vehicles'] = 2
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0.0
    total_cost = 0.0
    cost_per_meter = 1.5
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Vehicle {vehicle_id}\'s route: '
        route_distance = 0.0
        while not routing.IsEnd(index):
            plan_output += f'{manager.IndexToNode(index)} -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        route_cost = route_distance * cost_per_meter
        plan_output += f'{manager.IndexToNode(index)}'
        plan_output += f'\nDistance of the route: {route_distance:.2f}m'
        plan_output += f'\nCost of the route: £{route_cost:.2f}\n'
        print(plan_output)
        total_distance += route_distance
        total_cost += route_cost
    print(f'Total distance of all routes: {total_distance:.2f}m')
    print(f'Total cost of all routes: £{total_cost:.2f}')

def main():

    data = create_data_model()
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):

        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    def demand_callback(from_index):

        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data['vehicle_capacities'],
        True,
        'Capacity')

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')

if __name__ == '__main__':
    main()