from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np


def create_data_model(distance_matrix, demands, vehicle_capacity):
    """存储问题的数据。"""
    data_model = {}
    data_model['distance_matrix'] = distance_matrix
    data_model['demands'] = demands
    data_model['vehicle_capacity'] = vehicle_capacity
    data_model['num_vehicles'] = len(vehicle_capacity)
    data_model['depot'] = 0  # 假设第一个位置是仓库
    return data_model


def print_solution(data, manager, routing, solution):
    """在控制台上打印解决方案。"""
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
    print(f'Total transportation cost: {total_transportation_cost}元')


# 创建并注册转移回调
def distance_callback(from_index, to_index, data_model,manager):
    """返回两个节点之间的距离。"""
    # 将从路由模型中的索引转换为距离矩阵中的索引
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    # 检查索引是否在距离矩阵的范围内
    if from_node < len(data_model['distance_matrix']) and to_node < len(data_model['distance_matrix']):
        return data_model['distance_matrix'][from_node][to_node]
    return 0  # 对于虚拟的结束节点，返回0



def main():
    """程序的入口点。"""

    distance_matrix = [
        [0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7, 14.1, 15.6, 17.0, 18.4, 19.8],
        [1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7, 14.1, 15.6, 17.0, 18.4],
        [2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7, 14.1, 15.6, 17.0],
        [4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7, 14.1, 15.6],
        [5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7, 14.1],
        [7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3, 12.7],
        [8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9, 11.3],
        [9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5, 9.9],
        [11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1, 8.5],
        [12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7, 7.1],
        [14.1, 12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2, 5.7],
        [15.6, 14.1, 12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8, 4.2],
        [17.0, 15.6, 14.1, 12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4, 2.8],
        [18.4, 17.0, 15.6, 14.1, 12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0, 1.4],
        [19.8, 18.4, 17.0, 15.6, 14.1, 12.7, 11.3, 9.9, 8.5, 7.1, 5.7, 4.2, 2.8, 1.4, 0.0]
    ]

    demands = [0, 389, 370, 359, 341, 327, 326, 290, 265, 262, 261, 261, 248, 220, 220]  # 每个节点的需求量
    vehicle_capacity = [2000 ,1139, 1000 ]  # 每辆车的最大容量

    # 创建数据模型
    data_model = create_data_model(distance_matrix, demands, vehicle_capacity)

    # 创建路由索引管理器
    manager = pywrapcp.RoutingIndexManager(len(data_model['distance_matrix']), data_model['num_vehicles'],
                                           data_model['depot'])

    # 创建路由模型
    routing = pywrapcp.RoutingModel(manager)

    # 修正：使用闭包来传递额外的参数到 distance_callback
    def create_distance_callback(data_model, manager):
        """创建一个距离回调函数的闭包。"""

        def distance_callback(from_index, to_index):
            """返回两个节点之间的距离。"""
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            if from_node < len(data_model['distance_matrix']) and to_node < len(data_model['distance_matrix']):
                return data_model['distance_matrix'][from_node][to_node]
            return 0
        return distance_callback

    # 使用闭包创建转移回调
    transit_callback = create_distance_callback(data_model, manager)
    transit_callback_index = routing.RegisterTransitCallback(transit_callback)

    # 定义每条弧的成本
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 添加车辆容量约束
    def demand_callback(from_index):
        """返回节点的需求量。"""
        from_node = manager.IndexToNode(from_index)
        return data_model['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # 车辆的初始容量为0
        data_model['vehicle_capacity'],  # 车辆的最大容量
        True,  # 从0开始计算累积量
        'Capacity')

    # 设置车辆的起始位置
    for i in range(data_model['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            routing.GetDimensionOrDie('Capacity').CumulVar(routing.Start(i)))  # 设置目标为最小化总运输成本

    # 设置搜索参数为局部搜索策略
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(10)  # 设置超时时间为10秒

    # 解决问题
    solution = routing.SolveWithParameters(search_parameters)

    # 在控制台上打印解决方案
    if solution:
        print_solution(data_model, manager, routing, solution)


if __name__ == '__main__':
    main()
