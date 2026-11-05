#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
circle_panel.py —— cos_shape circle_motion 节点的调参面板（wxPython）

通过 topic 在线调整圆周运动参数（均在下一圈生效），并提供启停按钮。
容器内 DDS 对短进程不可靠，本面板的 publisher 全部长驻；点"应用"时
每个量发两遍（间隔 100ms）作为冗余。

用法:
    ros2 run cos_shape circle_panel.py        # 需先启动仿真与 circle_motion 节点
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Point

import wx

TOPIC_NS = "/circle_motion"


class PanelNode(Node):
    """长驻 ROS2 节点：持有所有 publisher / subscriber。"""

    def __init__(self):
        super().__init__("circle_panel")
        qos = QoSProfile(depth=10)
        self.pub_enable = self.create_publisher(Bool, TOPIC_NS + "/enable", qos)
        self.pub_center = self.create_publisher(Point, TOPIC_NS + "/set_center", qos)
        self.pub_radius = self.create_publisher(Float64, TOPIC_NS + "/set_radius", qos)
        self.pub_angular = self.create_publisher(
            Float64, TOPIC_NS + "/set_angular_velocity", qos)
        self.pub_linear = self.create_publisher(
            Float64, TOPIC_NS + "/set_linear_velocity", qos)
        self.pub_incl = self.create_publisher(
            Float64, TOPIC_NS + "/set_inclination", qos)
        self.pub_azim = self.create_publisher(
            Float64, TOPIC_NS + "/set_azimuth", qos)
        self.last_state = ""
        self.create_subscription(
            String, TOPIC_NS + "/state", self._state_cb, qos)

    def _state_cb(self, msg):
        self.last_state = msg.data

    def send(self, publisher, msg):
        """冗余发送两次，对抗容器内偶发丢包。"""
        publisher.publish(msg)
        time.sleep(0.1)
        publisher.publish(msg)


class CirclePanelFrame(wx.Frame):
    def __init__(self, node):
        super().__init__(None, title="cos_shape · 圆周运动调参", size=(420, 520))
        self.node = node
        self.running = False

        panel = wx.Panel(self)
        grid = wx.FlexGridSizer(rows=0, cols=2, vgap=8, hgap=8)
        grid.AddGrowableCol(1, 1)

        def row(label, default):
            grid.Add(wx.StaticText(panel, label=label),
                     0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
            tc = wx.TextCtrl(panel, value=str(default))
            grid.Add(tc, 1, wx.EXPAND)
            return tc

        self.tc_cx = row("圆心 x [m]", "0.15")
        self.tc_cy = row("圆心 y [m]", "0.0")
        self.tc_cz = row("圆心 z [m]", "1.05")
        self.tc_radius = row("半径 [m]", "0.05")
        self.tc_speed = row("速度", "0.5")
        self.tc_incl = row("倾角 [deg]（0=水平 90=竖直）", "0.0")
        self.tc_azim = row("方位角 [deg]", "0.0")

        grid.Add(wx.StaticText(panel, label="速度模式"),
                 0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
        self.rb_mode = wx.RadioBox(
            panel, choices=["角速度 rad/s", "线速度 m/s"],
            majorDimension=1, style=wx.RA_SPECIFY_COLS)
        grid.Add(self.rb_mode, 1, wx.EXPAND)

        btn_apply = wx.Button(panel, label="应用参数（下一圈生效）")
        btn_apply.Bind(wx.EVT_BUTTON, self.on_apply)
        self.btn_toggle = wx.Button(panel, label="启动")
        self.btn_toggle.SetBackgroundColour(wx.Colour(200, 255, 200))
        self.btn_toggle.Bind(wx.EVT_BUTTON, self.on_toggle)

        self.st_state = wx.StaticText(panel, label="state: （暂无）")
        self.st_state.Wrap(380)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(grid, 0, wx.ALL | wx.EXPAND, 12)
        sizer.Add(btn_apply, 0, wx.ALL | wx.EXPAND, 12)
        sizer.Add(self.btn_toggle, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)
        sizer.Add(self.st_state, 0, wx.ALL | wx.EXPAND, 12)
        panel.SetSizer(sizer)

        # 周期刷新 state 显示
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(300)

    @staticmethod
    def _f(tc):
        return float(tc.GetValue())

    def on_apply(self, _evt):
        try:
            cx, cy, cz = self._f(self.tc_cx), self._f(self.tc_cy), self._f(self.tc_cz)
            radius = self._f(self.tc_radius)
            speed = self._f(self.tc_speed)
            incl = math.radians(self._f(self.tc_incl))
            azim = math.radians(self._f(self.tc_azim))
        except ValueError:
            wx.MessageBox("存在无法解析的数字，请检查输入。", "输入错误", wx.ICON_WARNING)
            return

        n = self.node
        n.send(n.pub_center, Point(x=cx, y=cy, z=cz))
        n.send(n.pub_radius, Float64(data=radius))
        n.send(n.pub_incl, Float64(data=incl))
        n.send(n.pub_azim, Float64(data=azim))
        if self.rb_mode.GetSelection() == 0:
            n.send(n.pub_angular, Float64(data=speed))
        else:
            n.send(n.pub_linear, Float64(data=speed))
        n.get_logger().info("参数已下发")

    def on_toggle(self, _evt):
        self.running = not self.running
        self.node.send(self.node.pub_enable, Bool(data=self.running))
        if self.running:
            self.btn_toggle.SetLabel("停止")
            self.btn_toggle.SetBackgroundColour(wx.Colour(255, 200, 200))
        else:
            self.btn_toggle.SetLabel("启动")
            self.btn_toggle.SetBackgroundColour(wx.Colour(200, 255, 200))
        self.btn_toggle.Refresh()

    def on_timer(self, _evt):
        if self.node.last_state:
            self.st_state.SetLabel("state: " + self.node.last_state)


def main():
    rclpy.init()
    node = PanelNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    app = wx.App()
    frame = CirclePanelFrame(node)
    frame.Show()
    try:
        app.MainLoop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
