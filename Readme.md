# 上位机

## 功能：

1. 数据通信
   
   基于rosBridge和QWebSocket进行ros和windows端qt软件的数据桥接，需解析与生成相关的JSON消息。

2. 机器人基本信息显示

	从ros话题获取机器人位姿信息等。

3. 数字孪生显示

	QGraphicsView显示机器人模型

## TODO:
1. 话题数据解析（能够解析，待加入更多功能）
2. 机器人信息显示
3. **接收"MediumSize/BodyHub/ServoPositions"话题数据，该话题不能持续输出，只有当程序退出后，才会显示中间的一些话题数据**
	暂定解决方案：修改BodyHubNode.cpp中的话题发布逻辑，使其在running状态持续发布话题
	![alt text](<image/ServoPositons pub.png>)
4. 3D模型显示

## 当前实现：

1.通信连接

在roban上新开终端启动ros_bridge，qt程序上通过QWebSocket进行连接

```c++
 roslaunch rosbridge_server rosbridge_websocket.launch
```
点击主界面连接设置，进入连接设置界面，输入连接网络ipv4地址和端口号（ros_bridge默认端口9090），可通过添加将数据保存至数据库中，选中表单中的项，可对数据进行删除。

勾选需要连接的ip前的复选框，点击连接即可建立通信。连接成功，主界面左下方的状态栏显示已连接。
![alt text](image/connect_setting.png)

2.电量显示
状态栏显示电池电量

3.IMU数据显示
右侧信息栏显示机器人位姿信息，包括oritention, linear_acceleration,angular_velocity

4.相机图像显示
实时接收机器人相机数据进行显示