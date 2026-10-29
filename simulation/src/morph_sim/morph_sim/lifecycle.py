"""Keep the ROS context valid until application cleanup has completed."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions


def initialize():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def spin(node):
    stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    previous={signum:signal.signal(signum,stop) for signum in (signal.SIGINT,signal.SIGTERM)}
    try:
        while rclpy.ok() and not stopped:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        # Further terminal signals must not interrupt destroy_node/log closing.
        for signum in previous:signal.signal(signum,signal.SIG_IGN)
