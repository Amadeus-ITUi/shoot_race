# 比赛流程调参

所有常用参数集中在 [`race_params.py`](race_params.py)，修改后重启主程序生效。

参数按阶段分为：

- 导航点与流程时间；
- shoot1 固定环形靶；
- shoot2 旋转靶；
- shoot3 移动靶；
- Moonshot 任务板识别；
- move_base 后的精对位；
- 障碍 B 直接控制。

常见调整：

```python
# shoot1 横向像素容差
SHOOT1_HORIZONTAL_TOLERANCE = 15

# shoot2 横向/纵向像素容差
SHOOT2_HORIZONTAL_TOLERANCE = 15
SHOOT2_VERTICAL_TOLERANCE = 15

# shoot3 机器人对准移动靶中心左/中/右侧的偏移量（米）
SHOOT3_REGION_X_OFFSET = {
    1: -0.12,
    2: 0.0,
    3: 0.12,
}

# 最终位置和朝向容差
FINE_POSITION_TOLERANCE = 0.010
FINE_YAW_TOLERANCE = radians(3)
```

单位约定：

- 坐标、位置误差和偏移：米；
- 线速度：米/秒；
- 角速度：弧度/秒；
- 图像误差：像素；
- 超时：秒；
- 导航点第三项：角度；
- 程序内部角度容差：弧度。
