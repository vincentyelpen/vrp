from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def create_data_model():
    """创建数据模型"""
    data = {}
    data['distance_matrix'] = [
        [0.0, 16.5, 15.4, 13.5, 15.5, 38.0, 9.4, 29.6, 20.8, 28.4, 11.7, 8.2, 25.5, 4.6, 22.1],
        [16.5, 0.0, 15.4, 32.7, 9.6, 13.8, 35.2, 26.2, 26.0, 57.2, 11.5, 23.8, 65.8, 12.6, 34.7],
        [15.4, 15.4, 0.0, 42.8, 9.8, 26.5, 33.7, 14.8, 10.0, 46.2, 21.6, 20.0, 66.6, 16.0, 45.4],
        [13.5, 32.7, 42.8, 0.0, 37.0, 25.5, 6.8, 53.6, 53.4, 15.8, 25.4, 10.3, 28.1, 13.8, 8.9],
        [15.5, 9.6, 9.8, 37.0, 0.0, 20.6, 39.6, 20.6, 20.2, 40.4, 15.8, 28.1, 60.8, 16.9, 39.4],
        [38.0, 13.8, 26.5, 25.5, 20.6, 0.0, 28.0, 56.4, 67.0, 36.8, 8.9, 19.6, 52.1, 14.8, 28.3],
        [9.4, 35.2, 33.7, 6.8, 39.6, 28.0, 0.0, 45.4, 31.0, 23.5, 11.4, 4.0, 21.2, 7.6, 15.2],
        [29.6, 26.2, 14.8, 53.6, 20.6, 56.4, 45.4, 0.0, 13.4, 88.6, 32.5, 32.8, 101.5, 29.0, 56.0],
        [20.8, 26.0, 10.0, 53.4, 20.2, 67.0, 31.0, 13.4, 0.0, 99.2, 32.2, 29.3, 98.8, 25.0, 55.8],
        [28.4, 57.2, 46.2, 15.8, 40.4, 36.8, 23.5, 88.6, 99.2, 0.0, 28.8, 22.2, 30.9, 26.4, 7.3],
        [11.7, 11.5, 21.6, 25.4, 15.8, 8.9, 11.4, 32.5, 32.2, 28.8, 0.0, 8.0, 49.2, 7.3, 28.2],
        [8.2, 23.8, 20.0, 10.3, 28.1, 19.6, 4.0, 32.8, 29.3, 22.2, 8.0, 0.0, 33.8, 4.7, 18.1],
        [25.5, 65.8, 66.6, 28.1, 60.8, 52.1, 21.2, 101.5, 98.8, 30.9, 49.2, 33.8, 0.0, 27.6, 31.3],
        [4.6, 12.6, 16.0, 13.8, 16.9, 14.8, 7.6, 29.0, 25.0, 26.4, 7.3, 4.7, 27.6, 0.0, 14.0],
        [22.1, 34.7, 45.4, 8.9, 39.4, 28.3, 15.2, 56.0, 55.8, 7.3, 28.2, 18.1, 31.3, 14.0, 0.0]
    ]
    data['demands'] = [0, 1, 1, 2, 4, 2, 1, 4, 8, 8, 1, 2, 1, 2, 4]
    data['vehicle_capacities'] = [21, 21]
    data['num_vehicles'] = 2
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    """打印解决方案"""
    total_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total Distance of all routes: {}m'.format(total_distance))

def main():
    """解决VRP问题"""
    data = create_data_model()

    # 创建路由索引管理器。
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # 创建Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # 创建并注册一个传递回调函数。
    def distance_callback(from_index, to_index):
        # 返回两个节点之间的距离。
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 定义成本的每条边
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 添加容量约束。
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # 设置启发式。
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # 求解路径规划。
    solution = routing.SolveWithParameters(search_parameters)

    # 打印解决方案。
    if solution:
        print_solution(data, manager, routing, solution)

if __name__ == '__main__':
    main()
