import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq
from collections import defaultdict
import time
from scipy.interpolate import make_interp_spline  # 新增B样条依赖

# ====================== B样条曲线优化 ======================
def smooth_path_with_bspline(path, num_points=100, degree=3):
    """使用B样条平滑路径"""
    if len(path) < 2:
        return path
    
    # 转换为numpy数组
    path_array = np.array(path)
    
    # 生成参数化t值
    t = np.linspace(0, 1, len(path_array))
    
    # 创建B样条对象
    t_new = np.linspace(0, 1, num_points)
    
    try:
        # 三次B样条插值
        spl_x = make_interp_spline(t, path_array[:,0], k=degree)
        spl_y = make_interp_spline(t, path_array[:,1], k=degree)
        spl_z = make_interp_spline(t, path_array[:,2], k=degree)
        
        # 生成新路径点（关键修正）
        smoothed = np.column_stack((spl_x(t_new), spl_y(t_new), spl_z(t_new)))  # 注意双重括号
        return smoothed.tolist()  # 取消取整
    except ValueError:
        return path

def get_3d_neighbors(node, x_max, y_max, z_max, obstacles, closed_set):
    """26邻域扩展（含对角线）及动态代价计算"""
    directions = [(dx, dy, dz) 
                 for dx in (-1, 0, 1) 
                 for dy in (-1, 0, 1) 
                 for dz in (-1, 0, 1) 
                 if (dx, dy, dz) != (0, 0, 0)]
    
    neighbors = []
    x, y, z = node
    
    for dx, dy, dz in directions:
        nx = x + dx
        ny = y + dy
        nz = z + dz
        
        # 边界检查（1-based坐标）
        if 1 <= nx <= x_max and 1 <= ny <= y_max and 1 <= nz <= z_max:
            neighbor = (nx, ny, nz)
            if neighbor not in obstacles and neighbor not in closed_set:
                # 计算移动代价（欧氏距离）
                move_cost = np.sqrt(dx**2 + dy**2 + dz**2)
                neighbors.append((neighbor, move_cost))
    
    return neighbors

def heuristic_3d(a, b, weight=1.0):
    """带权重的三维对角线距离"""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    dz = abs(a[2] - b[2])
    return weight * (dx + dy + dz)  # 可替换为欧氏距离：np.sqrt(dx**2 + dy**2 + dz**2)

def a_star_3d_optimized(start, goal, obstacles, x_size, y_size, z_size, weight=1.0):
    """优化后的三维A*算法"""
    # 初始化数据结构
    open_heap = []
    heapq.heappush(open_heap, (0, start))
    
    came_from = dict()
    g_score = defaultdict(lambda: np.inf)
    g_score[start] = 0
    
    closed_set = set()
    obstacles_set = set(obstacles)
    
    # 预计算启发式值
    h_start = heuristic_3d(start, goal, weight)
    f_score = defaultdict(lambda: np.inf)
    f_score[start] = h_start

    while open_heap:
        current_f, current = heapq.heappop(open_heap)
        
        if current == goal:
            # 路径重构
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        
        if current in closed_set:
            continue
        closed_set.add(current)
        
        # 获取邻域节点及移动代价
        neighbors = get_3d_neighbors(current, x_size, y_size, z_size, obstacles_set, closed_set)
        for neighbor, move_cost in neighbors:
            tentative_g = g_score[current] + move_cost
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = heuristic_3d(neighbor, goal, weight)
                f = tentative_g + h
                if f < f_score[neighbor]:
                    heapq.heappush(open_heap, (f, neighbor))
                    f_score[neighbor] = f
    
    return None  # 无路径

# ====================== 分层搜索策略 ====================== 
def hierarchical_a_star(start, goal, obstacles, x_size, y_size, z_size, levels=2):
    """分层搜索优化"""
    # 层级1：粗粒度搜索（缩放地图）
    scale_factor = 2
    coarse_start = tuple((np.array(start)-1)//scale_factor + 1)
    coarse_goal = tuple((np.array(goal)-1)//scale_factor + 1)
    coarse_obstacles = set(tuple((np.array(obs)-1)//scale_factor + 1) for obs in obstacles)
    
    # 执行粗粒度搜索
    coarse_path = a_star_3d_optimized(coarse_start, coarse_goal, coarse_obstacles, 
                                    x_size//scale_factor, y_size//scale_factor, z_size//scale_factor)
    
    if not coarse_path:
        return None
    
    # 层级2：细粒度搜索（局部优化）
    refined_path = []
    for i in range(len(coarse_path)-1):
        sub_start = tuple((np.array(coarse_path[i])-1)*scale_factor + 1)
        sub_goal = tuple((np.array(coarse_path[i+1])-1)*scale_factor + 1)
        local_path = a_star_3d_optimized(sub_start, sub_goal, obstacles, x_size, y_size, z_size)
        if not local_path:
            return None
        refined_path.extend(local_path[:-1])
    
    refined_path.append(goal)
    return refined_path

def plot_3d_map(start, goal, obstacles, path=None, smoothed_path=None):  # 修改函数签名
    """优化后的三维可视化（增加平滑路径显示）"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制障碍物
    if obstacles:
        obs_array = np.array(list(obstacles))
        if obs_array.size > 0:
            ax.scatter(obs_array[:,0], obs_array[:,1], obs_array[:,2], 
                       c='blue', marker='s', s=50, alpha=0.3, depthshade=False)
    
    # 起点终点
    ax.scatter(*start, c='lime', s=200, marker='*', depthshade=False)
    ax.scatter(*goal, c='red', s=200, marker='X', depthshade=False)
    
    # 原始路径
    if path:
        path_array = np.array(path)
        ax.plot(path_array[:,0], path_array[:,1], path_array[:,2], 
                'gray', linestyle='--', linewidth=1, alpha=0.5, label='原始路径')
    
    # 平滑路径
    if smoothed_path:  # 新增平滑路径绘制
        smooth_array = np.array(smoothed_path)
        ax.plot(smooth_array[:,0], smooth_array[:,1], smooth_array[:,2], 
                'r-', linewidth=2, markersize=6, label='平滑路径')
        
    # 图例和样式
    ax.legend()
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(True, linestyle=':', alpha=0.5)
    plt.tight_layout()
    plt.show()

# ====================== 使用示例 ======================
if __name__ == "__main__":
    # 参数配置
    X_SIZE, Y_SIZE, Z_SIZE = 20, 20, 20
    START = (1, 1, 1)
    GOAL = (20, 20, 20)
    
    # 生成随机障碍物
    OBSTACLES = set()
    while len(OBSTACLES) < 0.1 * X_SIZE*Y_SIZE*Z_SIZE:
        node = (np.random.randint(1, X_SIZE+1), 
               np.random.randint(1, Y_SIZE+1), 
               np.random.randint(1, Z_SIZE+1))
        if node != START and node != GOAL:
            OBSTACLES.add(node)
    
    # 分层搜索路径
    print("\n=== 分层搜索优化 ===")
    start_time = time.perf_counter()
    path_hier = hierarchical_a_star(START, GOAL, OBSTACLES, X_SIZE, Y_SIZE, Z_SIZE)
    print(f"耗时: {time.perf_counter()-start_time:.2f}s")
    
    # 添加路径存在性检查
    if path_hier:
        # B样条平滑
        smoothed_path = smooth_path_with_bspline(path_hier, num_points=100, degree=3)
        # 可视化
        plot_3d_map(START, GOAL, OBSTACLES, path_hier, smoothed_path)
    else:
        print("警告：未找到可行路径，跳过可视化")  # 新增提示信息
    
    # # B样条平滑
    # if path_hier:
    #     smoothed_path = smooth_path_with_bspline(path_hier, num_points=100, degree=3)
    # else:
    #     smoothed_path = None
    
    # # 可视化（同时显示原始和平滑路径）
    # plot_3d_map(START, GOAL, OBSTACLES, path_hier, smoothed_path)