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