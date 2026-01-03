# SPDX-FileCopyrightText: 2025 Komiya Takumi
# SPDX-License-Identifier: Apache-2.0

import unittest
import rclpy
from std_msgs.msg import Float32
import launch
import launch_ros.actions
import launch_testing.actions
import pytest

@pytest.mark.rostest
def generate_test_description():
    # テスト対象のノードを起動する設定
    watchdog_node = launch_ros.actions.Node(
        package='ros2_system_watchdog',
        executable='watchdog',
        name='test_watchdog',
    )

    return (
        launch.LaunchDescription([
            watchdog_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'watchdog_node': watchdog_node}
    )

class TestWatchdogOutput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_listener')

    def tearDown(self):
        self.node.destroy_node()

    def test_topic_output(self):
        # /cpu_usage トピックからデータが流れてくるかチェック
        msgs_received = []
        sub = self.node.create_subscription(
            Float32,
            '/cpu_usage',
            lambda msg: msgs_received.append(msg),
            10
        )

        # 最大10秒間、メッセージを待つ
        end_time = rclpy.clock.Clock().now().nanoseconds + 10e9
        while rclpy.ok() and len(msgs_received) < 1:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if rclpy.clock.Clock().now().nanoseconds > end_time:
                break

        # メッセージが1つ以上受信できているか確認（入出力テスト）
        self.assertGreaterEqual(len(msgs_received), 1, "Topic /cpu_usage did not receive any messages.")
        self.assertIsInstance(msgs_received[0].data, float)
        self.assertTrue(0.0 <= msgs_received[0].data <= 100.0)
