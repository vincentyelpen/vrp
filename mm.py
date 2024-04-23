
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():

    data = {}

    data['distance_matrix'] = [
        [0, 16500, 15400, 13500, 15500, 38000, 9400, 29600, 20800, 28400, 11700, 8200, 25500, 4600, 22100],
        [16500, 0, 15400, 32700, 9600, 13800, 35200, 26200, 26000, 57200, 11500, 23800, 65800, 12600, 34700],
        [15400, 15400, 0, 42800, 9800, 26500, 33700, 14800, 10000, 46200, 21600, 20000, 66600, 16000, 45400],
        [13500, 32700, 42800, 0, 37000, 25500, 6800, 53600, 53400, 15800, 25400, 10300, 28100, 13800, 8900],
        [15500, 9600, 9800, 37000, 0, 20600, 39600, 20600, 20200, 40400, 15800, 28100, 60800, 16900, 39400],
        [38000, 13800, 26500, 25500, 20600, 0, 28000, 56400, 67000, 36800, 8900, 19600, 52100, 14800, 28300],
        [9400, 35200, 33700, 6800, 39600, 28000, 0, 45400, 31000, 23500, 11400, 4000, 21200, 7600, 15200],
        [29600, 26200, 14800, 53600, 20600, 56400, 45400, 0, 13400, 88600, 32500, 32800, 101500, 29000, 56000],
        [20800, 26000, 10000, 53400, 20200, 67000, 31000, 13400, 0, 99200, 32200, 29300, 98800, 25000, 55800],
        [28400, 57200, 46200, 15800, 40400, 36800, 23500, 88600, 99200, 0, 28800, 22200, 30900, 26400, 7300],
        [11700, 11500, 21600, 25400, 15800, 8900, 11400, 32500, 32200, 28800, 0, 8000, 49200, 7300, 28200],
        [8200, 23800, 20000, 10300, 28100, 19600, 4000, 32800, 29300, 22200, 8000, 0, 33800, 4700, 18100],
        [25500, 65800, 66600, 28100, 60800, 52100, 21200, 101500, 98800, 30900, 49200, 33800, 0, 27600, 31300],
        [4600, 12600, 16000, 13800, 16900, 14800, 7600, 29000, 25000, 26400, 7300, 4700, 27600, 0, 14000],
        [22100, 34700, 45400, 8900, 39400, 28300, 15200, 56000, 55800, 7300, 28200, 18100, 31300, 14000, 0]
    ]
    data['demands'] = [0, 389, 370, 359, 341, 327, 326, 290, 265, 262, 261, 261, 248, 220, 220]
    num_vehicles = 5
    data['vehicle_capacities'] = [1000] * num_vehicles
    #一辆车容量 4139
#两辆车容量 2402
# 3 1470
# 4 1284
# 5 1000-1100

    data['num_vehicles'] = num_vehicles
    data['depot'] = 0
    return data

# def print_solution(data, manager, routing, solution):
#
#     total_distance = 0
#     total_cost = 0
#     cost_per_meter = 1.5
#     for vehicle_id in range(data['num_vehicles']):
#         index = routing.Start(vehicle_id)
#         plan_output = f'Vehicle {vehicle_id}\'s route: '
#         route_distance = 0
#         while not routing.IsEnd(index):
#             plan_output += f'{manager.IndexToNode(index)} -> '
#             previous_index = index
#             index = solution.Value(routing.NextVar(index))
#             route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id) / 1000  # 将距离单位从毫米转换为米
#         route_cost = route_distance * cost_per_meter
#         plan_output += f'{manager.IndexToNode(index)}'
#         plan_output += f'\nDistance of the route: {route_distance:.2f}m'
#         plan_output += f'\nCost of the route: £{route_cost:.2f}\n'
#         print(plan_output)
#         total_distance += route_distance
#         total_cost += route_cost
#     print(f'Total distance of all routes: {total_distance:.2f}m')
#     print(f'Total cost of all routes: £{total_cost:.2f}')

def print_solution(data, manager, routing, solution):
    total_distance = 0
    total_cost = 0
    cost_per_demand = 0.0015

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Vehicle {vehicle_id}\'s route: '
        route_distance = 0
        route_cost = 0

        while not routing.IsEnd(index):
            from_node = manager.IndexToNode(index)
            plan_output += f'{from_node} -> '
            index = solution.Value(routing.NextVar(index))
            to_node = manager.IndexToNode(index)


            distance = data['distance_matrix'][from_node][to_node]
            demand = data['demands'][to_node]
            segment_cost = distance * demand

            route_distance += distance / 1000
            route_cost += segment_cost

        demand_cost = route_cost * cost_per_demand
        total_route_cost =  demand_cost

        plan_output += f'{to_node}'
        plan_output += f'\nDistance of the route: {route_distance:.2f}m'

        plan_output += f'\nCost of the route (per demand meter): £{demand_cost:.2f}'
        plan_output += f'\nTotal cost of the route: £{total_route_cost:.2f}\n'

        print(plan_output)

        total_distance += route_distance
        total_cost += total_route_cost

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
