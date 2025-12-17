# Ngăn xếp điều khiển ROS 2 cho Dobot Magician
<img src="https://img.shields.io/badge/ros--version-humble-green"/> <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/> [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
<img src="https://user-images.githubusercontent.com/80155305/212770939-2d7a4389-4143-4147-a50a-3453f73a1317.png" width="250" height="250"/><img src="https://user-images.githubusercontent.com/80155305/207256203-75f2607e-b40c-4a45-bf4e-de4aaffe6530.png" width="350" height="250"/><img src="https://user-images.githubusercontent.com/80155305/204082368-6eb63a16-2a22-4aed-83f8-45730a7b5d93.png" width="200" height="250"/>
</p>

## Mục lục :clipboard:
* [Các gói trong kho lưu trữ](#packages-vi)
* [Cài đặt](#installation-vi)
* [Khởi động hệ thống](#running-vi)
* [Quy trình về gốc (Homing)](#homing-vi)
* [Chẩn đoán](#diagnostics-vi)
* [Các chủ đề được xuất bản](#topics-vi)
* [Chuyển động](#motion-vi)
* [Trực quan hóa trong RViz](#visualization-vi)
* [Bảng điều khiển Dobot Magician - plugin RQT](#dmcp-vi)
* [Các đầu cuối (End effectors)](#end_effector_control-vi)
* [Công cụ bổ sung để trực quan hóa](#additional-vi)
* [Ví dụ](#examples-vi)
* [Thanh trượt](#rail-vi)
* [Hệ thống đa robot (MRS)](#mrs-vi)
* [Video - xem cách hệ thống hoạt động](#video-vi)
* [Câu hỏi thường gặp (FAQ)](#faq-vi)
* [Trích dẫn](#citing-vi)
* [Đóng góp](#contributing-vi)


<a name="packages-vi"></a>
## Các gói trong kho lưu trữ :open_file_folder:

  - `dobot_bringup` - các tệp launch và cấu hình tham số (trong các tệp _YAML_)
  - `dobot_control_panel` - plugin RQT để điều khiển cánh tay robot Dobot Magician (cũng như thanh trượt)
  - `dobot_demos` - một bộ sưu tập các kịch bản mẫu cho người mới bắt đầu (_ví dụ hoạt động tối thiểu_)
  - `dobot_description` - gói chứa mô tả URDF của Dobot Magician cùng với các lưới (meshes)
  - `dobot_diagnostics` - tổng hợp và phân tích các trạng thái cảnh báo
  - `dobot_driver` - giao diện Python cấp thấp để giao tiếp với Dobot qua cổng nối tiếp
  - `dobot_end_effector` - bộ các máy chủ dịch vụ cho phép điều khiển các loại đầu cuối khác nhau
  - `dobot_homing` - công cụ để thực hiện quy trình về gốc cho các động cơ bước của Dobot
  - `dobot_kinematics` - triển khai động học thuận và nghịch để xác thực tính khả thi của quỹ đạo
  - `dobot_motion` - gói chứa máy chủ hành động để điều khiển chuyển động của Dobot Magician (chuyển động nội suy khớp / chuyển động tuyến tính)
  - `dobot_msgs` - gói định nghĩa các thông điệp được sử dụng bởi ngăn xếp điều khiển
  - `dobot_state_updater` - gói chứa một nút thường xuyên truy xuất thông tin về trạng thái của robot (ví dụ: góc khớp / vị trí TCP)
  - `dobot_visualization_tools` - các công cụ hữu ích để trực quan hóa (ví dụ: quỹ đạo / phạm vi) dưới dạng các điểm đánh dấu RViZ

<a name="installation-vi"></a>
## Cài đặt :arrow_down:

### Yêu cầu

Hệ thống điều khiển này yêu cầu một thiết lập hệ thống với ROS 2. Khuyến nghị sử dụng Ubuntu 22.04 với [ROS 2 Humble](https://docs.ros.org/en/humble/index.html), tuy nhiên sử dụng Ubuntu 20.04 với [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html) cũng sẽ hoạt động.

### Cài đặt ROS 2 Humble Hawksbill
Thực hiện theo các hướng dẫn từ [liên kết](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Có 3 phiên bản ROS 2 Humble Hawksbill để lựa chọn: Desktop Install, ROS-Base Install và Development tools Install. Hãy chắc chắn cài đặt phiên bản **Desktop Install** (`sudo apt install ros-humble-desktop`).


### Cài đặt các mô-đun và gói bổ sung
Tất cả các mô-đun cần thiết đều có trong [requirements.txt](https://github.com/jkaniuka/magician_ros2/blob/main/requirements.txt), cài đặt bằng cách sử dụng: `pip3 install -r requirements.txt`
Các gói từ kho apt: `sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl`
:warning: Sau khi cài đặt các plugin RQT mới, hãy chạy `rqt --force-discover` để các plugin hiển thị trong giao diện đồ họa RQT. Vấn đề này được mô tả thêm [ở đây](https://answers.ros.org/question/338282/ros2-what-is-the-rqt-force-discover-option-meaning/).

### Tạo không gian làm việc cho hệ thống điều khiển (xây dựng từ nguồn)
```
source /opt/ros/humble/setup.bash
mkdir -p ~/magician_ros2_control_system_ws/src
git clone https://github.com/jkaniuka/magician_ros2.git ~/magician_ros2_control_system_ws/src
cd magician_ros2_control_system_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

### Truy cập vào các cổng nối tiếp
Để giao tiếp với robot, cần có quyền truy cập vào các cổng nối tiếp. Để có thể mở các cổng nối tiếp mà không cần sử dụng `sudo`, bạn cần thêm mình vào nhóm **dialout**:

```bash
sudo usermod -a -G dialout <tên_người_dùng>
# Yêu cầu đăng nhập lại hoặc khởi động lại!
```

:warning: Cổng USB mà Dobot Magician được kết nối mặc định là **/dev/ttyUSB0** - bạn có thể thay đổi nó trong [tệp này](./dobot_driver/dobot_driver/dobot_handle.py).

<a name="running-vi"></a>
## Khởi động hệ thống :robot:
1. Kết nối Dobot Magician với máy tính bằng cáp USB và sau đó bật nó lên.
2. Đặt biến môi trường MAGICIAN_TOOL mô tả cấu hình của robot `export MAGICIAN_TOOL=<loại_công_cụ>` (các giá trị được phép là: _none, pen, suction_cup, gripper, extended_gripper_).
3. Từ bên trong thư mục **magician_ros2_control_system_ws**, chạy `. install/setup.bash` để kích hoạt không gian làm việc của bạn.
3. Khởi chạy toàn bộ ngăn xếp điều khiển với `ros2 launch dobot_bringup dobot_magician_control_system.launch.py`.


<a name="homing-vi"></a>
## Quy trình về gốc (Homing)
Việc về gốc nên được thực hiện như hành động đầu tiên sau khi hệ thống được khởi động. Điều này là cần thiết vì một bộ mã hóa gia tăng đã được đặt ở đế của máy thao tác, và robot không nhận biết được vị trí thực tế của nó khi được cấp nguồn. Dừng tất cả các kịch bản điều khiển robot khác trước khi bắt đầu quy trình về gốc.
Việc về gốc được xử lý bởi máy chủ dịch vụ, để bắt đầu nó, hãy chạy lệnh sau:
```
ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure
```
Một tham số _homing\_position_ được gán cho nút máy chủ của dịch vụ về gốc, cho phép bạn xác định vị trí mà máy thao tác đạt được sau khi quy trình hoàn tất. Dưới đây là các ví dụ về lệnh để đọc và đặt giá trị tham số.
```
ros2 param get /dobot_homing_srv homing_position
ros2 param set /dobot_homing_srv homing_position [150.0,0.0,100.0,0.0]
```

<a name="diagnostics-vi"></a>
## Chẩn đoán
Hệ thống chăm sóc chẩn đoán trạng thái của robot. Sử dụng plugin RQT _Diagnostics Viewer_, bạn có thể trực quan hóa rõ ràng thông tin về các trạng thái cảnh báo đang hoạt động. Để khởi động _Diagnostics Viewer_, hãy chạy lệnh sau:
```
rqt -s rqt_robot_monitor
```
Nếu bạn đã mở RQT, bạn sẽ tìm thấy plugin này trong phần _Plugins -> Robot Tools -> Diagnostics Viewer_. Sau khi mở plugin, hãy chọn tùy chọn _Alternative view_ ở trên cùng. Sau khi nhấp đúp vào mô tả cảnh báo, một cửa sổ sẽ xuất hiện với thông tin về nguyên nhân có thể và cách xóa nó. Dưới đây bạn sẽ tìm thấy ảnh chụp màn hình của plugin _Diagnostics Viewer_.
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220293202-d1320648-719d-4e3c-a592-52e8607d3838.png" width="400" height="400"/><img src="https://user-images.githubusercontent.com/80155305/220293214-8e07c4ef-67fa-40c1-a562-7c97e81730ff.png" width="275" height="400"/>
</p>

<a name="topics-vi"></a>
## Các chủ đề được xuất bản
- `/joint_states` (sensor_msgs/msg/JointState) - giá trị góc trong các khớp của máy thao tác
- `/dobot_TCP` (geometry_msgs/msg/PoseStamped) - vị trí của hệ tọa độ gắn với đầu của liên kết robot cuối cùng (hướng được đưa ra dưới dạng một quaternion)
- `/dobot_pose_raw` (std_msgs/msg/Float64MultiArray) - vị trí của hệ tọa độ gắn với đầu của liên kết robot cuối cùng (hướng thô nhận được từ Dobot, được biểu thị bằng độ)

<a name="motion-vi"></a>
## Chuyển động
Chuyển động của máy thao tác được xử lý bằng hành động ROS 2. Để máy thao tác di chuyển đến vị trí mong muốn, mục tiêu chuyển động phải được gửi đến máy chủ hành động. Phần sau mô tả cấu trúc của thông điệp được gửi đến máy chủ hành động (một phần của _dobot\_msgs/action/PointToPoint_):
* **motion_type**:
  * 1 -> chuyển động nội suy khớp, mục tiêu được biểu thị bằng tọa độ Descartes
  * 2 -> chuyển động tuyến tính, mục tiêu được biểu thị bằng tọa độ Descartes
  * 4 -> chuyển động nội suy khớp, mục tiêu được biểu thị bằng tọa độ khớp
  * 5 -> chuyển động tuyến tính, mục tiêu được biểu thị bằng tọa độ khớp
* **target_pose** - vị trí mong muốn được biểu thị bằng tọa độ Descartes [_mm_] hoặc bằng tọa độ khớp [_độ_]
* **velocity_ratio** (mặc định 1.0)
* **acceleration_ratio** (mặc định 1.0)

Một ví dụ về lệnh cho phép bạn gửi một mục tiêu đến máy chủ hành động có thể được tìm thấy bên dưới (thêm cờ `--feedback` sẽ khiến terminal hiển thị vị trí hiện tại của robot trong khi nó đang di chuyển):
```
ros2 action send_goal /PTP_action dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [200.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback
```
Nếu bạn muốn hủy mục tiêu, hãy chạy lệnh sau:
```
ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal
```
Các tham số liên quan đến nút máy chủ hành động chuyển động cho phép bạn xác định vận tốc và gia tốc chuyển động của các khớp riêng lẻ của robot (`JT1_vel, JT2_vel, JT3_vel, JT4_vel, JT1_acc, JT2_acc, JT3_acc, JT4_acc`), cũng như các tham số của chuyển động tuyến tính của máy thao tác (`TCP_vel, end_tool_rot_vel, TCP_acc, end_tool_rot_acc`).


### Xác thực mục tiêu/quỹ đạo
Mục tiêu chuyển động được kiểm tra bởi máy chủ dịch vụ xác thực quỹ đạo trước khi thực hiện. Máy chủ dịch vụ xác thực quỹ đạo kiểm tra xem điểm mục tiêu có nằm trong không gian làm việc của máy thao tác hay không và có giải pháp cho bài toán động học nghịch hay không.

Các tham số của nút thực hiện máy chủ xác thực quỹ đạo là: `axis_1_range`, `axis_2_range`, `axis_3_range`, `axis_4_range`.
* Bốn tham số đầu tiên này cho phép không gian làm việc của máy thao tác bị giới hạn bằng cách đặt các giới hạn vị trí tại các khớp khác với những giới hạn do thiết kế cơ khí. Nếu bạn gửi một lệnh chuyển động đến một điểm vi phạm các giới hạn bạn đã xác định, bạn sẽ nhận được phản hồi sau từ máy chủ hành động PointToPoint:
```
Response: [PTP_server-1] [WARN] [1668081940.281544573] [dobot_PTP_server]:
Goal rejected: dobot_msgs.srv.EvaluatePTPTrajectory_Response(is_valid=False,
message='Joint limits violated')
```


<a name="visualization-vi"></a>
## Trực quan hóa trong RViz
Trong Rviz, bạn có thể hiển thị một trong tối đa 8 cấu hình robot khác nhau. Tất cả các cấu hình được phép được đặt trong sơ đồ dưới đây:
![image](https://user-images.githubusercontent.com/80155305/212382287-59fe1bd2-4e2e-4e00-ac8e-f1dd359ecaee.png)
Lệnh khởi động trực quan hóa máy thao tác trong cấu hình ví dụ như sau:
```
ros2 launch dobot_description display.launch.py DOF:=4 tool:=extended_gripper use_camera:=true
```
Nếu robot bị ngắt kết nối khỏi máy tính, bạn có thể bắt đầu trực quan hóa bằng cách thêm đối số `gui:=true` và điều khiển robot bằng nút [**joint_state_publisher_gui**](https://index.ros.org/p/joint_state_publisher_gui/).

Dưới đây bạn sẽ tìm thấy 3 ví dụ trực quan hóa:
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212384408-b57bfde4-b837-4513-8ea3-89a7bffd97af.png" width="260" height="260"/><img src="https://user-images.githubusercontent.com/80155305/212382237-fe2eadbf-19bc-4e6c-86f4-3a0e06edccf4.png" width="260" height="260"/><img src="https://user-images.githubusercontent.com/80155305/212383859-5afbccb4-dc07-4a80-a705-3a7af915c96e.png" width="260" height="260"/>
</p>


<a name="dmcp-vi"></a>
## Bảng điều khiển Dobot Magician
Bảng điều khiển Dobot Magician là một plugin RQT cho phép bạn định vị máy thao tác một cách thuận tiện, trực quan hóa trạng thái của nó, sửa đổi các tham số chuyển động và điều khiển các đầu cuối. Để khởi chạy nó, hãy chạy lệnh sau:
```
rqt -s dobot_control_panel
```
:warning: Khi bạn sử dụng _Bảng điều khiển Dobot Magician_, không có chương trình điều khiển robot nào khác có thể được chạy. Robot được điều khiển ở **chế độ thủ công** hoặc ở **chế độ tự động**.
Dưới đây bạn sẽ tìm thấy ảnh chụp màn hình của tất cả các màn hình plugin:

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220214176-0c0844c0-447b-481c-941b-aafd72b94d5a.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/220214179-14b4493f-4dd0-4de5-89fc-d8c60e9d4d8b.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/220214183-296eac2b-a3b0-4a84-b799-145440944eef.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp
</p>

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220214189-8eb57aca-bed8-4b55-a3fc-25244a41e721.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/212384115-3875164a-3717-465e-a3cc-cda9b8913ea6.png" width="250" height="230"/>
</p>

<a name="end_effector_control-vi"></a>
## Điều khiển đầu cuối (End Effector)
Việc điều khiển kẹp và cốc hút khí nén đã được thực hiện bằng các dịch vụ ROS 2.

### Kẹp (Gripper):
Ví dụ lệnh:
```
ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: 'open', keep_compressor_running: true}"
```
Các trường yêu cầu:
- `gripper state` (kiểu _string_) : _open/close_ (mở/đóng)
- `keep_compressor_running` (kiểu _bool_) : _true/false_ (đúng/sai)

### Cốc hút (Suction cup):
Ví dụ lệnh:
```
ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"
```
Các trường yêu cầu:
- `enable_suction` (kiểu _bool_) : _true/false_ (đúng/sai)


<a name="additional-vi"></a>
## Công cụ bổ sung để trực quan hóa
Gói **dobot_visualization_tools** cung cấp các công cụ trực quan hóa dưới dạng các điểm đánh dấu RViz.
1. Trường nhìn của camera (FOV) cho Intel Realsense D435i (được xuất bản trên chủ đề `/realsense_FOV`)
```
ros2 run dobot_visualization_tools show_camera_FOV
```
2. Trực quan hóa không gian làm việc (được xuất bản trên chủ đề `/reachability_range`) - phạm vi không có đầu cuối được gắn vào.
```
ros2 run dobot_visualization_tools show_dobot_range
```
3. Quỹ đạo TCP (được xuất bản trên chủ đề `/TCP_trajectory`) - các điểm đánh dấu quỹ đạo được loại bỏ sau 2 giây trạng thái đứng yên của máy thao tác, để không làm chậm RViz với quá nhiều điểm đánh dấu cần hiển thị.
```
ros2 run dobot_visualization_tools show_trajectory
```
4. Điểm đánh dấu tương tác (Interactive Markers) - bạn có thể chọn vị trí mục tiêu của đầu công cụ (x, y, z, r) bằng cách di chuyển điểm đánh dấu tương tác trong RViz. Sau khi chọn mục tiêu, nhấp chuột phải vào quả cầu màu vàng và chọn loại chuyển động từ menu. Khi chạy một nút cho phép điều khiển bằng các điểm đánh dấu tương tác, phải chỉ định hai tham số (xem ví dụ bên dưới) xác định điểm ở đầu công cụ trong hệ tọa độ được liên kết với liên kết cuối cùng của máy thao tác.
```
ros2 run dobot_visualization_tools int_marker --ros-args -p TCP_x_offset:=0.059 -p TCP_z_offset:=-0.12
```

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212383385-e9c2ca38-8cfa-4ac9-a106-99d2663ce629.png" width="220" height="220"/><img src="https://user-images.githubusercontent.com/80155305/212384560-198f5b65-b248-428d-baad-bf75c6909542.png" width="330" height="220"/>
</p>

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212383482-6fa7b2c5-1bc1-4f0a-a118-a93dbc788a94.png" width="250" height="250"/><img src="https://user-images.githubusercontent.com/80155305/220213781-8e1b593d-4865-424c-9631-87724acba8d6.png" width="250" height="250"/>
</p>

<video controls width="250">
    <source src="https://user-images.githubusercontent.com/80155305/219739487-8edd727b-aee9-4f14-b7c3-1d6c78ce2d4d.mp4">
</video>


<a name="examples-vi"></a>
## Ví dụ
Các kịch bản mẫu đã được đưa vào gói **dobot_demos**. Phân tích các mã mẫu (_ví dụ hoạt động tối thiểu_) từ gói này sẽ giúp bạn hiểu cách điều khiển robot Dobot Magician.
Chạy các kịch bản mẫu:
- `ros2 run dobot_demos test_gripper`
- `ros2 run dobot_demos test_suction_cup`
- `ros2 run dobot_demos test_homing`
- `ros2 run dobot_demos test_point_to_point`
- `ros2 run dobot_demos test_pick_and_place`


<a name="rail-vi"></a>
## Thanh trượt
Nếu bạn có _Thanh trượt Dobot Magician_ và bạn muốn nhận phản hồi thời gian thực về vị trí của xe trượt, bạn cần xuất biến môi trường `MAGICIAN_RAIL_IN_USE` trước khi khởi chạy toàn bộ ngăn xếp điều khiển (`export MAGICIAN_RAIL_IN_USE=true`). Vị trí hiện tại của xe trượt trên thanh trượt sẽ được xuất bản trên chủ đề `/dobot_rail_pose` ở tần số 20 Hz. Sau khi ngắt kết nối thanh trượt, hãy gõ `unset MAGICIAN_RAIL_IN_USE` và khởi động lại toàn bộ hệ thống điều khiển.

Việc điều khiển thanh trượt có thể thực hiện được cả từ plugin RQT **Bảng điều khiển Dobot Magician** và bằng cách sử dụng máy chủ hành động **/move_sliding_rail**. Để khởi chạy máy chủ hành động **/move_sliding_rail** và tải các tham số (vận tốc và gia tốc của thanh trượt), hãy sử dụng lệnh dưới đây:
```
ros2 launch dobot_motion dobot_rail.launch.py
```
Một ví dụ về lệnh cho phép bạn gửi một mục tiêu đến máy chủ hành động có thể được tìm thấy bên dưới:
```
ros2 action send_goal /move_sliding_rail dobot_msgs/action/SlidingRail "{target_pose: 500}"
```

<a name="mrs-vi"></a>
## Hệ thống đa robot (MRS) :robot:
Nếu bạn muốn kết nối nhiều robot Dobot Magician vào cùng một máy tính hoặc vào cùng một mạng, do đó tạo ra một _hệ thống đa robot_ (MRS), hãy kiểm tra nhánh [**magician-mrs**](https://github.com/jkaniuka/magician_ros2/tree/magician-mrs).

<a name="video-vi"></a>
## Video - xem cách hệ thống hoạt động :movie_camera:

| Tổng quan và các tính năng | Điểm đánh dấu tương tác |
| ------ | ------ |
| [<img src="https://user-images.githubusercontent.com/80155305/215295789-e6b1dadd-a819-4fe9-a633-4ce483a47964.png" width="100%">](https://vimeo.com/793400746) | <video src="https://github.com/jkaniuka/dobot_tmp/assets/80155305/7b946f88-e4c8-499f-84f4-1ecf10534b25">|




<a name="faq-vi"></a>
## Câu hỏi thường gặp :question:
**Tại sao tôi không sử dụng MoveIt 2?**

MoveIt 2 là một công cụ tuyệt vời, nhưng tôi đã không sử dụng nó trong hệ thống điều khiển của mình vì nhiều lý do:
* Để bắt đầu, hệ thống này được tạo ra với ý tưởng sử dụng nó khi học ROS 2. Đầu tiên, bạn cần làm quen với các thực thể như hành động, chủ đề, dịch vụ, tham số để sau đó hiểu cách chúng được sử dụng trong MoveIt 2.
* Thứ hai, MoveIt được sử dụng trong số những thứ khác để tạo ra các quỹ đạo không va chạm. Thiết kế cơ khí của robot Dobot Magician và phạm vi rất ngắn của nó (32 cm) làm cho nó _de facto_ không phù hợp để làm việc trong không gian có chướng ngại vật.
* Cuối cùng, MoveIt cho phép tạo ra các kiểu cầm nắm. Dobot Magician được trang bị một kẹp có thể mở hoàn toàn hoặc đóng hoàn toàn. Ngoài ra, nó luôn hướng xuống theo chiều ngang. Việc tạo ra kiểu cầm nắm trong trường hợp này dường như không cần thiết.

<a name="citing-vi"></a>
## Trích dẫn :scroll:
Nếu bạn thấy công việc này hữu ích, vui lòng ghi công cho tác giả bằng cách trích dẫn:

```
@mastersthesis{jkaniuka-bsc-23-twiki,
  author = {Kaniuka, Jan},
  school = {WEiTI},
  title = {{System sterowania robota manipulacyjnego Dobot Magician na bazie frameworka ROS2}},
  engtitle = {{Control system of DobotMagician robotic manipulator based on ROS2 framework}},
  year = {2023},
  type = {Bachelor's thesis},
  lang = {pl},
  organization = {IAiIS},
  publisher = {IAiIS},
  tutor = {Tomasz Winiarski},
  twiki = {bsc},
  url = {https://gitlab-stud.elka.pw.edu.pl/robotyka/rpmpg_pubs/-/raw/main/student-theses/jkaniuka-bsc-23-twiki.pdf}
}
```

<a name="contributing-vi"></a>
## Đóng góp

#### Báo cáo lỗi & Yêu cầu tính năng

Vui lòng sử dụng [**trình theo dõi sự cố**](https://github.com/jkaniuka/magician_ros2/issues) để báo cáo bất kỳ lỗi hoặc yêu cầu tính năng nào.