from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np


def create_data_model(distance_matrix, demands, vehicle_capacity):
    """Stores the data for the problem."""
    data_model = {}
    data_model['distance_matrix'] = distance_matrix
    data_model['demands'] = demands
    data_model['vehicle_capacity'] = vehicle_capacity
    data_model['num_vehicles'] = len(vehicle_capacity)
    data_model['depot'] = 0  # 假设第一个位置是仓库
    return data_model


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            plan_output += f' {node_index} -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if index == routing.Start(vehicle_id):
                break
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += f'{manager.IndexToNode(index)}\n'
        plan_output += f'Distance of the route for vehicle {vehicle_id}: {route_distance}m\n'
        print(plan_output)
        total_distance += route_distance
    print(f'Total distance of all routes: {total_distance}m')


def main():
    """Entry point of the program."""

    distance_matrix = [
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

    demands = [0, 389, 370, 359, 341, 327, 326, 290, 265, 262, 261, 261, 248, 220, 220]  # 每个节点的需求量
    vehicle_capacity = [2000,2000,2000]  # 每辆车的最大容量,,,,,更改车的数量

    # 创建数据模型
    data_model = create_data_model(distance_matrix, demands, vehicle_capacity)

    # 创建路由索引管理器
    manager = pywrapcp.RoutingIndexManager(len(data_model['distance_matrix']), data_model['num_vehicles'],
                                           data_model['depot'])

    # 创建路由模型
    routing = pywrapcp.RoutingModel(manager)

    # 创建并注册转移回调
    def distance_callback(from_index, to_index):
        # 返回两个节点之间的距离
        return data_model['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 定义每条弧的成本
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 设置车辆的最大行驶距离
    routing.AddDimension(transit_callback_index, 0, 1000, True, 'Distance')
    distance_dimension = routing.GetDimensionOrDie('Distance')
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # 添加车辆容量约束
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return data_model['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data_model['vehicle_capacity'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # 设置车辆的起始位置
    depot = data_model['depot']
    for i in range(data_model['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(distance_dimension.CumulVar(manager.NodeToIndex(depot)))

    # 设置搜索参数
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # 解决问题
    solution = routing.SolveWithParameters(search_parameters)

    # 在控制台上打印解决方案
    if solution:
        print_solution(data_model, manager, routing, solution)

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_transportation_cost = 0  # 总运输成本
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        route_distance = 0
        route_transportation_cost = 0  # 当前车辆的运输成本
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            plan_output += f' {node_index} -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if index == routing.Start(vehicle_id):
                break
            arc_distance = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            route_distance += arc_distance
            # 计算当前节点的运输成本
            demand = data['demands'][node_index]
            transportation_cost = arc_distance * 1.5 * demand  # 每个货物每km1.5元
            route_transportation_cost += transportation_cost
        plan_output += f'{manager.IndexToNode(index)}\n'
        plan_output += f'Distance of the route for vehicle {vehicle_id}: {route_distance}m\n'
        plan_output += f'Transportation cost of the route for vehicle {vehicle_id}: {route_transportation_cost}元\n'
        print(plan_output)
        total_distance += route_distance
        total_transportation_cost += route_transportation_cost
    print(f'Total distance of all routes: {total_distance}m')
    print(f'Total transportation cost: {total_transportation_cost}')



if __name__ == '__main__':
    main()