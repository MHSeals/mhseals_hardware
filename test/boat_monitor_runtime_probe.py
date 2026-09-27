"""Actual ROS adapter check. Run only network-isolated, ROS_DOMAIN_ID=231.

No hardware launch, GPIO, PWM, or cmd_vel publisher is created.
"""
import os
import tempfile
import time
import subprocess
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage

from mhseals_hardware.boat_test import BoatTest, build_parser
from mhseals_hardware.configuration import configure_args


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    help_result = subprocess.run([sys.executable, '-m', 'mhseals_hardware.boat_test', '--help'],
                                 capture_output=True, text=True, check=True)
    assert '--config' in help_result.stdout and '--monitor-only' in help_result.stdout
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix='boat-monitor-probe-') as output:
        forbidden_config = Path(output) / 'must-not-write.yaml'
        forbidden = subprocess.run([
            sys.executable, '-m', 'mhseals_hardware.boat_test', '--monitor-only',
            '--save-config', '--config', str(forbidden_config)], capture_output=True, text=True)
        assert forbidden.returncode == 2 and not forbidden_config.exists(), forbidden.stderr
        args = configure_args(build_parser(), ['--bag-output', output + '/test'])
        test = BoatTest(args)
        node = rclpy.create_node('boat_monitor_fixture')
        odom_pub = node.create_publisher(Odometry, args.odom_topic, 10)
        imu_pub = node.create_publisher(Imu, '/imu/raw', 10)
        gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)
        tf_pub = node.create_publisher(TFMessage, '/tf', 10)
        try:
            start = time.monotonic()
            while time.monotonic() - start < 2:
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.orientation.w = 1.
                odom.twist.twist.linear.x = .2
                odom_pub.publish(odom)
                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = 'base_link'
                imu.orientation.w = 1.
                imu_pub.publish(imu)
                gps = NavSatFix()
                gps.header.stamp = stamp
                gps.status.status = 0
                gps.position_covariance_type = 2
                gps.latitude, gps.longitude = 42., -71.
                gps_pub.publish(gps)
                transforms = []
                for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
                    transform = TransformStamped()
                    transform.header.stamp = stamp
                    transform.header.frame_id = parent
                    transform.child_frame_id = child
                    transform.transform.rotation.w = 1.
                    transforms.append(transform)
                tf_pub.publish(TFMessage(transforms=transforms))
                rclpy.spin_once(node, timeout_sec=.02)
            snapshot = test.monitor.snapshot()
            for name in ('odometry', 'IMU', 'GPS'):
                assert snapshot['sensors'][name]['status'] == 'LIVE', snapshot
                assert snapshot['sensors'][name]['hz'] > 10, snapshot
            assert all(row['status'] == 'LIVE' for row in snapshot['tf']), snapshot
            assert snapshot['sensors']['odometry']['values']['vx'] == .2
            assert not test.armed and test.command_publisher is None and not test.processes
            report = test.save_report()
            assert report.is_file()
            time.sleep(args.stale_after + .2)
            assert test.monitor.snapshot()['sensors']['odometry']['status'] == 'STALE'
            print('PASS actual ROS subscriptions, TF callback, velocities, freshness, JSON export; passive/no actuators')
        finally:
            test.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
