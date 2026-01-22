# UAV 行为树系统使用指南

本文档介绍如何为 BP_UAV_Default 添加和配置行为树。

---

## 使用步骤

### 步骤 1: 编译项目

1. 关闭 Unreal Editor（如果已打开）
2. 在 Visual Studio 中打开项目
3. 选择 `Development Editor` 配置
4. 按 `Ctrl+Shift+B` 或点击 `Build -> Build Solution` 编译
5. 等待编译完成，确保没有错误

### 步骤 2: 创建 AI 资产文件夹

在 Unreal Editor 的内容浏览器中：

1. 导航到 `Content/UAV/`
2. 右键点击空白处 -> **New Folder**
3. 命名为 `AI`
4. 进入 `Content/UAV/AI/` 文件夹

最终文件夹结构：
```
Content/
└── UAV/
    ├── AI/                    <- 新建此文件夹
    │   ├── BB_UAV_Default     <- 黑板资产
    │   └── BT_UAV_Default     <- 行为树资产
    └── Blueprints/
        └── UAVs/
            └── BP_UAV_Default
```

### 步骤 3: 创建黑板资产 (Blackboard)

1. 在 `Content/UAV/AI/` 文件夹中
2. 右键点击空白处 -> **Artificial Intelligence** -> **Blackboard**
3. 命名为 `BB_UAV_Default`
4. 双击打开黑板编辑器
5. 点击 **New Key** 按钮添加以下键值：

| 键名 | 类型 | 用途 |
|------|------|------|
| `TargetLocation` | Vector | 目标飞行位置 |
| `HomeLocation` | Vector | 起始/返航位置 |
| `TargetActor` | Object (Base Class: Actor) | 目标跟踪Actor |
| `PatrolIndex` | Int | 当前巡逻点索引 |
| `CurrentLocation` | Vector | UAV当前位置 |
| `CurrentVelocity` | Vector | UAV当前速度 |
| `CurrentAltitude` | Float | UAV当前高度 |

6. 保存黑板资产 (`Ctrl+S`)

### 步骤 4: 创建行为树资产 (Behavior Tree)

1. 在 `Content/UAV/AI/` 文件夹中
2. 右键点击空白处 -> **Artificial Intelligence** -> **Behavior Tree**
3. 命名为 `BT_UAV_Default`
4. 双击打开行为树编辑器
5. 在右侧 **Details** 面板中：
   - 找到 **Blackboard Asset** 属性
   - 点击下拉框，选择 `BB_UAV_Default`
6. 保存行为树资产 (`Ctrl+S`)

### 步骤 5: 创建 AI 控制器蓝图

1. 在 `Content/UAV/AI/` 文件夹中
2. 右键点击空白处 -> **Blueprint Class**
3. 在弹出窗口中，展开 **All Classes**
4. 搜索并选择 `UAVAIController`
5. 点击 **Select**
6. 命名为 `BP_UAVAIController`
7. 双击打开蓝图编辑器
8. 在 **Class Defaults**（类默认值）中设置：
   - **Behavior Tree Asset**: 选择 `BT_UAV_Default`
   - **Auto Start Behavior Tree**: ✓ 勾选
9. 编译并保存 (`Compile` -> `Save`)

### 步骤 6: 创建 UAVPawn 蓝图

由于现有的 `BP_UAV_Default` 可能继承自 `UAVActor`，我们需要创建新的蓝图或修改现有蓝图：

**方法 A: 创建新蓝图（推荐）**

1. 在 `Content/UAV/Blueprints/UAVs/` 文件夹中
2. 右键点击空白处 -> **Blueprint Class**
3. 展开 **All Classes**，搜索并选择 `UAVPawn`
4. 命名为 `BP_UAVPawn_Default`
5. 双击打开蓝图编辑器
6. 在 **Class Defaults** 中设置：
   - **AI Controller Class**: 选择 `BP_UAVAIController`
   - **Auto Possess AI**: 选择 `Placed in World or Spawned`
7. 添加无人机网格体组件（如果需要可视化）：
   - 在 Components 面板点击 **Add Component**
   - 选择 **Static Mesh**
   - 设置网格体为无人机模型
8. 编译并保存

**方法 B: 修改现有蓝图（需要重新父类化）**

1. 打开 `Content/UAV/Blueprints/UAVs/BP_UAV_Default`
2. 点击菜单 **File** -> **Reparent Blueprint**
3. 搜索并选择 `UAVPawn`
4. 确认重新父类化
5. 在 **Class Defaults** 中设置：
   - **AI Controller Class**: 选择 `BP_UAVAIController`
   - **Auto Possess AI**: 选择 `Placed in World or Spawned`
6. 编译并保存

### 步骤 7: 设计行为树

打开 `BT_UAV_Default`，设计行为树结构：

**示例 1: 简单悬停**
```
Root
└── Sequence
    └── UAV Hover
        - Hover Duration: 0 (无限悬停)
```

**示例 2: 飞往目标位置**
```
Root
└── Sequence
    └── UAV Fly To Location
        - Target Location Key: TargetLocation
        - Acceptable Radius: 100
```

**示例 3: 巡逻行为**
```
Root
└── Sequence (Loop)
    ├── UAV Get Next Patrol Point
    │   - Patrol Index Key: PatrolIndex
    │   - Target Location Key: TargetLocation
    │   - Patrol Points: [(0,0,500), (1000,0,500), (1000,1000,500), (0,1000,500)]
    │   - Loop Patrol: ✓
    ├── UAV Fly To Location
    │   - Target Location Key: TargetLocation
    │   - Acceptable Radius: 100
    └── UAV Hover
        - Hover Duration: 2.0
```

**添加节点方法：**
1. 右键点击 Root 节点
2. 选择 **Add Composite** -> **Sequence** 或 **Selector**
3. 右键点击 Composite 节点
4. 选择 **Add Task** -> 找到 `UAV` 开头的任务节点

### 步骤 8: 在关卡中放置 UAV

1. 打开你的关卡
2. 从内容浏览器拖拽 `BP_UAVPawn_Default` 到关卡中
3. 调整位置和旋转
4. 运行游戏测试

---

## 可用的行为树节点

### 任务节点 (Tasks)

#### UAV Fly To Location
让UAV飞往指定位置。
- **Target Location Key**: 目标位置的黑板键（Vector类型）
- **Acceptable Radius**: 到达判定距离（默认: 100 cm）
- **Fly Speed**: 飞行速度（默认: 500 cm/s）

#### UAV Hover
让UAV在当前位置悬停。
- **Hover Duration**: 悬停时间（秒），0表示无限悬停
- **Height Offset**: 高度偏移（相对于当前高度）

#### UAV Get Next Patrol Point
获取下一个巡逻点并设置到黑板。
- **Patrol Index Key**: 巡逻索引的黑板键（Int类型）
- **Target Location Key**: 目标位置的黑板键（Vector类型）
- **Patrol Points**: 巡逻点数组（在节点详情中编辑）
- **Loop Patrol**: 是否循环巡逻

### 服务节点 (Services)

#### Update UAV State
定期更新UAV状态到黑板。
- **Current Location Key**: 当前位置的黑板键
- **Current Velocity Key**: 当前速度的黑板键
- **Current Altitude Key**: 当前高度的黑板键
- **Interval**: 更新间隔（默认: 0.1秒）

**添加服务：** 右键点击 Composite 节点 -> **Add Service** -> **Update UAV State**

### 装饰器节点 (Decorators)

#### UAV At Location
检查UAV是否在指定位置。
- **Target Location Key**: 目标位置的黑板键
- **Acceptable Radius**: 可接受的距离误差

**添加装饰器：** 右键点击任务节点 -> **Add Decorator** -> **UAV At Location**

---

## 运行时控制

### 通过蓝图设置目标位置

在任意蓝图中：
1. 获取 UAVPawn 的引用
2. 调用 `Get Controller` 节点
3. Cast 到 `UAVAIController`
4. 调用 `Set Target Location` 或 `Set Target Actor`

### 通过 C++ 设置目标位置

```cpp
// 获取AI控制器
AUAVAIController* AIController = Cast<AUAVAIController>(UAVPawn->GetController());
if (AIController)
{
    // 设置目标位置
    AIController->SetTargetLocation(FVector(1000, 0, 500));
    
    // 或设置目标Actor
    AIController->SetTargetActor(TargetActor);
}
```

### 启动/停止行为树

```cpp
// 启动行为树
AIController->StartBehaviorTree();

// 停止行为树
AIController->StopBehaviorTree();
```

---

## 调试

### 查看行为树执行状态

1. 运行游戏
2. 在编辑器中选择场景中的 UAVPawn
3. 打开菜单 **Window** -> **Developer Tools** -> **Behavior Tree**
4. 可以实时查看：
   - 当前执行的节点（绿色高亮）
   - 黑板键值
   - 节点执行结果

### 查看黑板值

1. 在行为树调试器中
2. 点击左侧的 **Blackboard** 标签
3. 可以看到所有键的当前值

### 日志输出

行为树节点会输出日志到 Output Log：
- `LogTemp: BTTask_UAVPatrol: Moving to patrol point X at (X, Y, Z)`
- `LogTemp: UAVAIController: Behavior Tree started.`

---

## 文件结构总结

```
Content/
└── UAV/
    ├── AI/
    │   ├── BB_UAV_Default.uasset      # 黑板资产
    │   ├── BT_UAV_Default.uasset      # 行为树资产
    │   └── BP_UAVAIController.uasset  # AI控制器蓝图
    └── Blueprints/
        └── UAVs/
            ├── BP_UAV_Default.uasset      # 原有蓝图（可选保留）
            └── BP_UAVPawn_Default.uasset  # 新UAVPawn蓝图

Source/uav_simulator/
├── Core/
│   ├── UAVPawn.h/cpp              # UAVPawn C++类
│   └── UAVActor.h/cpp             # 原有UAVActor类
└── AI/
    ├── UAVAIController.h/cpp      # AI控制器
    ├── Tasks/
    │   ├── BTTask_UAVFlyToLocation.h/cpp
    │   ├── BTTask_UAVHover.h/cpp
    │   └── BTTask_UAVPatrol.h/cpp
    ├── Services/
    │   └── BTService_UAVUpdateState.h/cpp
    └── Decorators/
        └── BTDecorator_UAVAtLocation.h/cpp
```

---

## 常见问题

### Q: 行为树不执行？
A: 检查以下几点：
1. AI Controller Class 是否正确设置
2. Auto Possess AI 是否设置为 `Placed in World or Spawned`
3. Behavior Tree Asset 是否在 AI 控制器中正确设置
4. 黑板资产是否正确关联到行为树

### Q: UAV 不移动？
A: 检查以下几点：
1. 确保 `bUsePositionControl` 为 true
2. 检查目标位置是否正确设置到黑板
3. 查看 Output Log 是否有错误信息

### Q: 找不到 UAVPawn 类？
A: 确保项目已正确编译，并且 `uav_simulator.Build.cs` 中包含了 AIModule 依赖。
