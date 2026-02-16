# Planning 目录代码文件作用分析

## Source/uav_simulator/Planning

1. `Source/uav_simulator/Planning/PathPlanner.h`  
定义路径规划基类接口（`PlanPath`、障碍物管理、碰撞检测、结果可视化）和通用配置/状态字段。

2. `Source/uav_simulator/Planning/PathPlanner.cpp`  
实现基类通用逻辑：默认直线路径、点/线段碰撞检测、路径简化（可见性裁剪）、调试绘制与耗时记录。

3. `Source/uav_simulator/Planning/AStarPathPlanner.h`  
定义 A* 规划器和 A* 节点结构，声明网格化、邻居生成、启发式、边界设置等 A* 专用能力。

4. `Source/uav_simulator/Planning/AStarPathPlanner.cpp`  
实现 3D 网格 A* 搜索流程：开闭集搜索、碰撞过滤、路径回溯、起终点修正与简化。

5. `Source/uav_simulator/Planning/RRTPathPlanner.h`  
定义 RRT/RRT* 规划器和树节点结构，声明采样、最近点、扩展、重连等随机树算法接口。

6. `Source/uav_simulator/Planning/RRTPathPlanner.cpp`  
实现 RRT 主循环（随机采样→扩展→碰撞检测→到目标判定），并可选执行 RRT* 选父节点与重连优化。

7. `Source/uav_simulator/Planning/ObstacleManager.h`  
定义障碍物管理组件：注册/移除/查询、碰撞检测、最近障碍距离、动态更新与事件广播。

8. `Source/uav_simulator/Planning/ObstacleManager.cpp`  
实现障碍物生命周期管理、点/线碰撞算法、场景扫描注册、动态障碍同步和调试可视化。

9. `Source/uav_simulator/Planning/TrajectoryOptimizer.h`  
定义轨迹优化组件和多项式段结构，提供“航点→时间参数化轨迹”的生成与采样接口。

10. `Source/uav_simulator/Planning/TrajectoryOptimizer.cpp`  
实现时间分配、分段多项式系数求解、轨迹离散采样与状态查询（位置/速度/加速度）。

11. `Source/uav_simulator/Planning/TrajectoryTracker.h`  
定义轨迹跟踪组件：开始/暂停/恢复/停止、进度与完成状态、期望状态获取、误差计算、事件回调。

12. `Source/uav_simulator/Planning/TrajectoryTracker.cpp`  
实现按时间推进的跟踪状态机，在 Tick 中更新进度并插值轨迹点，触发完成/进度事件。

13. `Source/uav_simulator/Planning/TrajectoryData.h`  
定义 `UTrajectoryData` 数据容器（`UObject`），用于在系统间（如黑板/任务）传递轨迹。

14. `Source/uav_simulator/Planning/TrajectoryData.cpp`  
实现轨迹数据的设置、读取、有效性判断和清空。

15. `Source/uav_simulator/Planning/PlanningVisualizer.h`  
定义规划可视化组件接口：路径、轨迹、跟踪点、障碍物、搜索边界、航点、持久化绘制等。

16. `Source/uav_simulator/Planning/PlanningVisualizer.cpp`  
实现基于 `DrawDebug*` 的实际渲染逻辑，以及每帧重绘持久化路径/轨迹。

## Source/uav_simulator/Tests/Planning（测试对应关系）

1. `Source/uav_simulator/Tests/Planning/AStarPathPlannerTest.cpp`：验证 A* 的边界、避障、路径合理性。  
2. `Source/uav_simulator/Tests/Planning/RRTPathPlannerTest.cpp`：验证 RRT/RRT* 的可达性、避障、连续性。  
3. `Source/uav_simulator/Tests/Planning/ObstacleManagerTest.cpp`：验证障碍物注册、碰撞检测、距离/范围查询。  
4. `Source/uav_simulator/Tests/Planning/TrajectoryOptimizerTest.cpp`：验证时间分配、采样、约束与平滑性。  
5. `Source/uav_simulator/Tests/Planning/TrajectoryTrackerTest.cpp`：验证跟踪状态机、插值、误差与重置行为。

## 整体调用链（简版）

`ObstacleManager` 提供环境信息 → `AStar/RRT` 出路径 → `TrajectoryOptimizer` 出可执行轨迹 → `TrajectoryTracker` 按时间跟踪 → `PlanningVisualizer` 做调试显示。


## ASCII Visualization

### Module Graph

+-------------------+      +-------------------+
| ObstacleManager   |----->| PathPlanner(Base) |
+-------------------+      +-------------------+
                                   |
                    +--------------+--------------+
                    |                             |
            +-------------------+        +-------------------+
            | AStarPathPlanner  |        | RRTPathPlanner    |
            +-------------------+        +-------------------+
                    \                             /
                     \                           /
                      +-------------------------+
                      |  Path (Waypoints)       |
                      +-------------------------+
                                   |
                                   v
                      +-------------------------+
                      | TrajectoryOptimizer     |
                      +-------------------------+
                                   |
                                   v
                      +-------------------------+
                      | TrajectoryTracker       |
                      +-------------------------+
                                   |
                                   v
                      +-------------------------+
                      | UAV Control / Execution |
                      +-------------------------+

Side Utility:
+-------------------+
| PlanningVisualizer|
+-------------------+
Draws path/trajectory/obstacles/tracking points for debug.

### Runtime Flow (Simplified)

[Scan/Update Obstacles]
          |
          v
[Plan Path: A* or RRT]
          |
          v
[Path Simplify + Validate]
          |
          v
[Optimize to Trajectory]
          |
          v
[Start Tracking]
          |
          v
[Tick: desired state + progress]
          |
          v
[Collision warning?] --yes--> [Stop + Replan]
          |
          no
          v
[Reach Goal / Complete]
