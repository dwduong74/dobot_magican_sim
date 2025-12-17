# Dobot Magician ROS2 Simulation

Một gói mô phỏng ROS2 để điều khiển và tương tác với robot Dobot Magician trong Gazebo.

## Giới thiệu

Dự án này cung cấp một bộ các gói ROS2 để mô phỏng robot Dobot Magician. Nó bao gồm mô tả robot (URDF), bộ điều khiển Gazebo, các nút điều khiển chuyển động và các bản trình diễn để trình bày các khả năng của robot.

## Các tính năng

*   Mô phỏng robot Dobot Magician trong Gazebo.
*   Bộ điều khiển ROS2 để điều khiển khớp nối của robot.
*   Lập kế hoạch chuyển động và thực thi.
*   Các bản trình diễn ví dụ như gắp và đặt.
*   Bảng điều khiển RQt để điều khiển robot thủ công.

## Yêu cầu

*   ROS2 (Humble được khuyến nghị)
*   Gazebo
*   Python 3.8+
*   Colcon (công cụ xây dựng ROS2)
*   Các phụ thuộc Python khác:
    ```bash
    pip install -r src/requirements.txt
    ```

## Cài đặt và Xây dựng

1.  **Nhân bản kho lưu trữ:**
    ```bash
    git clone https://github.com/user/dobot_magican_sim.git
    cd dobot_magican_sim
    ```

2.  **Cài đặt các phụ thuộc ROS:**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Xây dựng không gian làm việc:**
    ```bash
    colcon build
    ```

## Cách sử dụng

1.  **Thiết lập môi trường:**
    Mỗi khi bạn mở một terminal mới, hãy tìm nguồn không gian làm việc của bạn:
    ```bash
    source install/setup.bash
    ```

2.  **Chạy mô phỏng:**
    Để khởi chạy mô phỏng Gazebo và RViz:
    ```bash
    ros2 launch dobot_description sim_and_view.launch.py
    ```
    Lệnh này sẽ khởi chạy Gazebo, sinh ra robot Dobot Magician và khởi động RViz để trực quan hóa.

3.  **Chạy bản trình diễn gắp và đặt:**
    Khi mô phỏng đang chạy, hãy mở một terminal mới khác, tìm nguồn không gian làm việc và chạy bản trình diễn:
    ```bash
    ros2 run dobot_demos pick_and_place.py
    ```

## Giấy phép

Dự án này được cấp phép theo Giấy phép MIT. Xem tệp `LICENSE` để biết chi tiết.