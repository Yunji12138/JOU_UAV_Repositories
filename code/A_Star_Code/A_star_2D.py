# 简单地图的A*算法实现
# 复杂地图的A*算法实现


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import math

# ======================== A*算法核心实现 ========================
def child_nodes_cal(current_node, rows, columns, obstacles, close_set):
    directions = [(-1,0), (1,0), (0,-1), (0,1)]  # 四邻域
    x, y = current_node
    neighbors = []
    
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 1 <= nx <= columns and 1 <= ny <= rows:  # 1-based坐标校验
            if (nx, ny) not in obstacles and (nx, ny) not in close_set:
                neighbors.append((nx, ny))
    return neighbors

def a_star_search(start, target, obstacles, rows, columns):
    open_heap = []
    heapq.heappush(open_heap, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    # 使用曼哈顿距离
    # f_score = {start: abs(start[0]-target[0]) + abs(start[1]-target[1])}
    # 使用欧几里得距离
    f_score = {start: math.sqrt((abs(start[0]-start[1]))^2 + (abs(target[0]-target[1]))^2)}
    close_set = set()
    obstacles_set = set(obstacles)

    while open_heap:
        current = heapq.heappop(open_heap)[1]
        
        if current == target:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]  # 反转路径
        
        close_set.add(current)
        
        for neighbor in child_nodes_cal(current, rows, columns, obstacles_set, close_set):
            tentative_g = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + (abs(neighbor[0]-target[0]) + abs(neighbor[1]-target[1]))
                heapq.heappush(open_heap, (f, neighbor))
    
    return None  # 无路径

# ======================== 可视化函数 ========================
def draw_grid(rows, columns, start, target, obstacles, path=None):
    fig, ax = plt.subplots(figsize=(10,10))
    
    # 绘制栅格线
    for i in range(rows+1):
        ax.axhline(i, color='k', lw=1)
    for j in range(columns+1):
        ax.axvline(j, color='k', lw=1)
    
    # 起点（绿色）
    ax.add_patch(patches.Rectangle(
        (start[0]-1, start[1]-1), 1, 1, 
        facecolor='lime', edgecolor='k')
    )
    
    # 终点（红色）
    ax.add_patch(patches.Rectangle(
        (target[0]-1, target[1]-1), 1, 1,
        facecolor='red', edgecolor='k')
    )
    
    # 障碍物（蓝色）
    for ob in obstacles:
        ax.add_patch(patches.Rectangle(
            (ob[0]-1, ob[1]-1), 1, 1,
            facecolor='blue', edgecolor='k')
        )
    
    # 路径（红色连线）
    if path:
        x_coords = [p[0]-0.5 for p in path]
        y_coords = [p[1]-0.5 for p in path]
        ax.plot(x_coords, y_coords, 'r-', lw=2, marker='o', markersize=8)
    
    # 坐标轴设置
    ax.set_xlim(0, columns)
    ax.set_ylim(0, rows)
    ax.set_aspect('equal')
    ax.invert_yaxis()  # 重要！Matlab坐标系与Matplotlib y轴方向相反
    plt.show()

# ======================== 主函数 ========================
def main():
    # ------------ 简单测试案例 ------------
    simple_obs = [(4,2), (4,3), (4,4)]
    simple_path = a_star_search(
        start=(2,3), 
        target=(6,3),
        obstacles=simple_obs,
        rows=5,
        columns=7
    )
    print("简单地图路径:", simple_path)
    draw_grid(
        rows=5, columns=7,
        start=(2,3), target=(6,3),
        obstacles=simple_obs,
        path=simple_path
    )
    
    # ------------ 复杂随机地图 ------------
    rows, columns = 25, 27
    start = (2, 3)
    target = (23, 23)
    obstacles = []
    
    # 生成随机障碍物（排除起点终点）
    while len(obstacles) < 250:
        node = (np.random.randint(1, columns+1), np.random.randint(1, rows+1))
        if node != start and node != target and node not in obstacles:
            obstacles.append(node)
    
    # 计算路径
    path = a_star_search(start, target, obstacles, rows, columns)
    print("复杂地图路径是否存在:", bool(path))
    
    # 可视化
    draw_grid(rows, columns, start, target, obstacles, path)

if __name__ == "__main__":
    main()