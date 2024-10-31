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

## 实现细节

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

## 多线程

1. **互斥锁 (`std::mutex`)**：
   - 使用 `std::mutex` 来保护共享资源（如 turtle_poses），确保在多线程环境下对共享资源的访问是线程安全的。
   - 例如，在 poseCallback 函数中使用 `std::lock_guard<std::mutex>` 来自动管理锁的生命周期。

2. **异步任务 (`std::async`)**：
   - 在 handleMoveToTarget 函数中，使用 `std::async` 来启动异步任务，执行 Move_To_Target

 函数。
   - 例如：
     ```cpp
     std::async(std::launch::async, Move_To_Target, turtle_name, target, std::ref(rate));
     ```
   - 这会在一个新的线程中异步执行 Move_To_Target 函数，而不会阻塞主线程。

3. **多线程的 ROS 服务服务器 (`ros::AsyncSpinner`)**：
   - 在 main 函数中，使用 `ros::AsyncSpinner` 来创建一个多线程的 ROS 服务服务器。
   - 例如：
     ```cpp
     ros::AsyncSpinner spinner(4);  // 使用4个线程
     spinner.start();
     ```
   - 这允许 ROS 在多个线程中处理回调函数，从而提高并发性和响应速度。

通过以上方式，代码实现了多线程处理，确保在处理多个乌龟的移动任务时能够高效且线程安全。
