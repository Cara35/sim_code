#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import tf
from tf.transformations import quaternion_from_euler
import math  # 导入 math 模块

msg = """
Control mbot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
}

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        print("Received key:", key)  # 添加打印语句来显示接收到的键盘输入
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('mbot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    tf_broadcaster = tf.TransformBroadcaster()

    # 初始化机器人的位置和姿态
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0

    try:
        print (msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print (vels(speed,turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            # 更新机器人的位置和方向
            robot_theta += turn * th  # 更新机器人的方向

            # 将线速度分解为 x 和 y 分量
            x_speed = speed * x * math.cos(robot_theta)
            y_speed = speed * x * math.sin(robot_theta)

            # 更新机器人的位置
            robot_x += x_speed * 0.1  # 假设每个按键操作后机器人在x方向上移动0.1
            robot_y += y_speed * 0.1  # 假设每个按键操作后机器人在y方向上移动0.1

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = x_speed
            twist.linear.y = y_speed
            twist.angular.z = turn * th

            # 打印发布的Twist消息内容
            print("Twist Message:", twist)

            # 发布Twist消息
            pub.publish(twist)

            # 打印TF变换
            print("TF Broadcast - Position:", (robot_x, robot_y, 0.0))
            print("TF Broadcast - Orientation (Euler):", (0, 0, robot_theta))

            # 发布base_footprint相对于odom的TF变换
            tf_broadcaster.sendTransform(
                (robot_x, robot_y, 0.0),  # 机器人在odom中的位置
                quaternion_from_euler(0, 0, robot_theta),  # 机器人的姿态，使用欧拉角转换为四元数 
                rospy.Time.now(),
                "base_footprint",
                "odom"
            )

    except Exception as e:
        print (e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
