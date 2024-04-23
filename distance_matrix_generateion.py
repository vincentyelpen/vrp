import numpy as np

def calculate_distance(point1, point2):
    # 计算两点之间的距离，这里使用欧几里得距离计算方法
    return round(np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2), 1)

def generate_distance_matrix(nodes):
    # 生成距离矩阵
    distance_matrix = []
    for node1 in nodes:
        row = []
        for node2 in nodes:
            distance = calculate_distance(node1, node2)
            row.append(distance)
        distance_matrix.append(row)
    return distance_matrix

def main():
    # 输入节点坐标
    nodes = [
        (0, 0), (1, 1), (2, 2), (3, 3), (4, 4),
        (5, 5), (6, 6), (7, 7), (8, 8), (9, 9),
        (10, 10), (11, 11), (12, 12), (13, 13), (14, 14)
    ]

    # 生成距离矩阵
    distance_matrix = generate_distance_matrix(nodes)

    # 输出距离矩阵
    print("distance_matrix = [")
    for row in distance_matrix:
        print("    [{}]".format(", ".join(map(str, row))))
    print("]")

if __name__ == "__main__":
    main()
