# Interactive Shape Drawer with ROS 2

## Overview
This project showcases an interactive application where users can input their desired shape (square, star, or triangle), and the application will draw the shape using the `turtlesim` simulator. All nodes are started via a launch file, and 2nd option started via user input node making the process seamless and efficient.

## Features
- **User Input for Shape Selection:** Prompts the user to choose a shape (square, star, or triangle) to draw.
- **Dynamic Node Launching:** Based on user input, the appropriate ROS 2 node is launched to draw the selected shape.
- **Turtlesim Integration:** Utilizes the `turtlesim` simulator to visualize the shapes being drawn.

## How It Works
1. **User Prompt:** When the application is launched, it prompts the user to select a shape.
2. **Shape Drawing:** Based on the user's input, the corresponding node is executed to draw the chosen shape in the `turtlesim` simulator.
3. **Launch File:** All nodes are managed and started via a single launch file.

## Usage
1. Clone the repository:
    ```sh
    git clone https://github.com/yashbhaskar/Interactive_Shape_Drawer_Application_With_ROS-2
    cd Interactive_Shape_Drawer_Application_With_ROS-2
    ```

2. Build the package:
    ```sh
    colcon build
    ```

3. Source your workspace:
    ```sh
    source install/setup.bash
    ```

4. Run the application:
    ```sh
    ros2 launch shapes_pkg main.py

    OR

    ros2 run shapes_pkg user.py
    ```

5. Follow the prompt to enter the shape you want to draw (`square`, `star`, or `triangle`).

## Files and Directories
- **scripts/**: Contains the Python scripts for drawing each shape (`square.py`, `star.py`, `triangle.py`).
- **launch/**: Contains the `main.py` launch file.
- **main.py**: The main script that handles user input and launches the appropriate node.

## Dependencies
- ROS 2 (Humble)
- `turtlesim` package

## Screenshots

### All Processes
![Screenshot 2024-08-06 114717](https://github.com/user-attachments/assets/43bd646d-db48-483c-86f2-a3c5074f62ce)

### rqt_graph
![Screenshot 2024-08-06 114812](https://github.com/user-attachments/assets/478df542-5595-4ade-8513-55e17bc18dc3)

## License
This project is licensed under the MIT License.
