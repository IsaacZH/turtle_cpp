# Turtle Draw

## 编译项目

1. 进入工作空间目录：
    ```sh
    cd turtle_draw
    ```

2. 使用 `catkin_make` 编译项目：
    ```sh
    catkin_make
    ```

3. 加载环境变量：
    ```sh
    source devel/setup.bash
    ```

## 运行 Launch 文件

运行 `dune.launch` 文件：
- C++版本
    ```sh
    roslaunch turtle_cpp dune.launch
    ```
- Python版本
    ```sh
    roslaunch turtle_py dune.launch
    ```

## 工程功能

这个工程的主要功能是使用ROS（机器人操作系统）和Turtlesim模拟器来控制多个乌龟的运动和行为。具体功能包括：
1. **生成乌龟**：生成多个乌龟并控制它们的运动。
2. **绘制图形**：通过控制乌龟的运动轨迹来绘制希尔伯特曲线。
3. **追逐行为**：一只乌龟追逐另一只乌龟。
4. **改变背景颜色**：定期改变模拟器背景颜色。
5. **数据发布**：发布乌龟之间的距离和朝向角度数据。

### 实现细节

#### 1. 生成乌龟
在 `turtle_Group4.cpp` 和 `turtle_chase.cpp` 文件中，通过调用 `/spawn` 服务生成新的乌龟。

#### 2. 绘制图形
在 `draw_helix.cpp` 文件中，通过递归生成希尔伯特曲线的点，然后调用 `MoveToTarget` 服务控制乌龟按顺序移动到这些点，从而绘制出希尔伯特曲线。

#### 3. 追逐行为
在 `turtle_chase.cpp` 文件中，通过订阅目标乌龟的位置，并调用 `MoveToTarget` 服务控制追逐乌龟不断移动到目标乌龟的位置，实现追逐行为。

#### 4. 改变背景颜色
在 `change_color.cpp` 文件中，通过定期设置背景颜色参数并调用 `/clear` 服务来改变模拟器的背景颜色。

#### 5. 数据发布
在 `data_scope_pub.cpp` 文件中，通过订阅两只乌龟的位置，计算它们之间的距离和朝向角度，并发布这些数据。
