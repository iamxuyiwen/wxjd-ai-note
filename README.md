# wxjd-ai-note
1. 连接Wi-Fi的详细流程
步骤1：进入机器人操作系统界面
假设巡检机器人运行Ubuntu系统，通过HDMI线连接显示器，或通过SSH远程登录机器人控制终端。

步骤2：查看可用Wi-Fi列表
打开终端，输入以下命令扫描周围Wi-Fi：


`nmcli dev wifi list`  # 查看可用Wi-Fi列表
输出示例：

SSID               MODE   CHAN  RATE       SIGNAL  BARS  SECURITY  
HG123_5G           Infra  44    540 Mbit/s  100     ▂▄▆█  WPA2      
HG123              Infra  6     195 Mbit/s  85      ▂▄▆_  WPA2      

步骤3：连接指定Wi-Fi
根据任务书提供的网络账号（SSID）及密码，执行以下命令连接：


`nmcli dev wifi connect <SSID名称> password "<密码>"`

示例（假设SSID为HG123，密码为12345678）：

`nmcli dev wifi connect HG123 password "12345678"`

步骤4：验证连接状态
查看是否成功获取IP地址：

`ifconfig wlan0`  # 无线网卡通常为wlan0或类似名称

输出示例（关键字段：inet 192.168.1.100）：

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
      inet 192.168.1.100  netmask 255.255.255.0  broadcast 192.168.1.255
      ...
      
步骤5：测试网络连通性
在机器人终端中ping智能编程训练平台的IP地址（假设为192.168.1.101）：

`ping 192.168.1.101 -c 4`  # 发送4个测试包

成功输出示例：

64 bytes from 192.168.1.101: icmp_seq=1 ttl=64 time=1.23 ms

64 bytes from 192.168.1.101: icmp_seq=2 ttl=64 time=1.05 ms

...

2. 图形界面操作（备用方案）
如果机器人支持桌面环境（如Ubuntu GNOME）：

点击右上角网络图标，选择目标Wi-Fi（SSID）。

输入密码，勾选“自动连接”。

连接成功后，图标变为扇形信号标志。

3. 键盘控制机器人运动

``` python

import keyboard
from robot_control import move_forward, move_backward, turn_left, turn_right

keyboard.add_hotkey('t', move_forward)
keyboard.add_hotkey('g', move_backward)
keyboard.add_hotkey('f', turn_left)
keyboard.add_hotkey('h', turn_right)
keyboard.wait('esc')  # 按ESC退出

```

4. 云台相机角度控制
1)命令示例（假设使用HTTP API）：
bash
curl -X POST http://<云台IP>/rotate?angle=180

2)调用相机驱动接口（串口或 ROS 服务）发送旋转命令。

示例 ROS 命令行：
rosservice call /gimbal_control/set_angle "{pitch: 0.0, yaw: 180.0}"


## 数据采集

编写脚本控制机器人移动至“导航点1”，循环不同角度、变焦拍照。

示例伪码（ROS + OpenCV）：

``` python

import cv2, os
from robot_api import RobotGimbal

gimbal = RobotGimbal()
cam = cv2.VideoCapture(0)
angles = [0, 45, 90, 135, 180]
focal_lengths = [0.8, 1.0, 1.2]

count = 1
os.makedirs('raw_images', exist_ok=True)
for ang in angles:
    gimbal.set_yaw(ang)
    for f in focal_lengths:
        cam.set(cv2.CAP_PROP_ZOOM, f)
        ret, frame = cam.read()
        if ret:
            cv2.imwrite(f'raw_images/img_{count:04d}.jpg', frame)
            count += 1

```

## 训练集数据划分：

``` python
import os, random

images = sorted(os.listdir('data/images'))
random.shuffle(images)
n = len(images)
train_n = int(n * 0.65)

with open('autosplit_train.txt','w') as f1, open('autosplit_val.txt','w') as f2:
    for img in images[:train_n]:
        f1.write(f'data/images/{img}\n')
    for img in images[train_n:]:
        f2.write(f'data/images/{img}\n')

```

``` python

import os, shutil
import random

# 原始图片与标签文件夹
img_dir = 'data/images'
label_dir = 'data/labels'

# 目标划分后文件夹
train_img = 'data_split/train/images'
train_lbl = 'data_split/train/labels'
val_img   = 'data_split/val/images'
val_lbl   = 'data_split/val/labels'

# 创建目录
for d in [train_img, train_lbl, val_img, val_lbl]:
    os.makedirs(d, exist_ok=True)

# 获取所有图片文件，打乱后划分
all_imgs = sorted(os.listdir(img_dir))
random.shuffle(all_imgs)
split_idx = int(len(all_imgs) * 0.65)
train_imgs = all_imgs[:split_idx]
val_imgs   = all_imgs[split_idx:]

# 复制图片与标签到对应文件夹
for img_list, tgt_img, tgt_lbl in [
    (train_imgs, train_img, train_lbl),
    (val_imgs,   val_img,   val_lbl)
]:
    for img_name in img_list:
        # 图片复制
        shutil.copy(os.path.join(img_dir, img_name), os.path.join(tgt_img, img_name))
        # 标签复制：同名 .txt
        lbl_name = os.path.splitext(img_name)[0] + '.txt'
        shutil.copy(os.path.join(label_dir, lbl_name), os.path.join(tgt_lbl, lbl_name))
```

# 激光雷达
激光雷达集成与可视化

```bash
# 启动雷达驱动（假设使用 ROS）
roslaunch rplidar_ros rplidar.launch
# 启动 rviz 并加载激光扫描话题
rosrun rviz rviz -d lidar_display.rviz

```
前后摄像头可视化
``` bash
# 启动摄像头节点
roslaunch usb_cam usb_cam-test.launch camera:=front
rosrun image_view image_view image:=/front/image_raw
# 后摄像头同理 camera:=rear
```

喇叭（扩音器）语音合成
```python

from robot_api import Speaker
from tts_tool import synthesize

sp = Speaker()
audio = synthesize("已到达导航点2")
sp.play(audio)
```
超声波模块可视化

```bash

# 发布超声波数据话题
rosrun ultrasonic_driver ultrasonic_node __name:=ultra
# Rviz 中添加 Marker 或 Range 显示
rosrun rviz rviz -d ultrasonic_display.rviz
```


## 模型训练及模型部署验证
实施思路与关键步骤

配置与训练 YOLOv11

修改配置文件 yolov11_cfg.yaml：

``` yaml
nc: 3                 # 类别数
epochs: 50            # 训练次数
batch: 16             # 批大小
data: data.yaml       # 指向 autosplit_train.txt / autosplit_val.txt
```

启动训练：

``` bash
python train.py --cfg yolov11_cfg.yaml --weights yolov11.pt --data data.yaml --device 0
```
验证单张图片检测

```bash
python detect.py --weights runs/train/exp/weights/best.pt \
                 --source test_images/sample.jpg --conf 0.25
```

在界面显示检测框和置信度。

部署到机器人

将 best.pt 放入机器人部署目录，调整部署脚本路径，运行实时检测程序：

```bash
python deploy_detect.py --model best.pt --camera 0
```

## 自动驾驶综合应用
任务C1. 建图与导航

实施思路与关键步骤

SLAM 建图

```bash
roslaunch karto_slam karto_mapping.launch
# 控制机器人在场地内来回移动以覆盖环境
```

地图保存：

```bash
rosrun map_server map_saver -f results/map_${选手编号}_${工位号}
```

设置导航点与自主导航

在 RViz 中点击“2D Nav Goal”标定导航点坐标，保存列表：

```yaml

nav_points:
  - name: pt0; x: 0.0; y: 0.0; theta: 0.0
  - name: pt1; x: 2.5; y: -1.0; theta: 1.57
```

导航程序伪码：

```python

import yaml
from robot_api import Navigator, Speaker

nav = Navigator()
sp = Speaker()
pts = yaml.safe_load(open('nav_points.yaml'))['nav_points']

for p in pts[:2]:
    nav.go_to(p['x'], p['y'], p['theta'])
    sp.play_text(f"已到达{p['name']}")
```

## 自动驾驶综合应用

实施思路与关键步骤

物流配送流程控制

编写上层流程逻辑（Python 伪码）：

```python

from robot_api import Navigator, GimbalCamera, Speaker

nav = Navigator(); cam = GimbalCamera(); sp = Speaker()

# 1. 点0→1
nav.go_to_point('pt0','pt1'); sp.play_text("已到达导航点1")
# 2. 物料识别
img = cam.capture(); label, conf = cam.detect(img)
sp.play_text(f"当前状态灯{label}亮, 置信度{conf:.2f}")
# 3. 点1→2
nav.go_to_point('pt1','pt2'); sp.play_text("已到达导航点2")
# 4. 道路标识识别与路线选择
sign, _ = cam.detect(nav.capture())
if sign == '向左转弯':
    order = ['pt2','pt0','pt1','pt2']
else:
    order = ['pt2','pt1','pt0','pt2']
# 5. 按顺序巡航
for i in range(len(order)-1):
    nav.go_to_point(order[i], order[i+1])
    sp.play_text(f"已到达{order[i+1]}")
```
