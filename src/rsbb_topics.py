#!/usr/bin/python
#topic types and corresponding names for the @Home referee box communication are listed here

from roah_rsbb_comm_ros.msg import Benchmark
from std_msgs.msg import Empty
from roah_rsbb_comm_ros.msg import DevicesState
from geometry_msgs.msg import Pose2D
from roah_rsbb_comm_ros.msg import TabletState
from std_msgs.msg import UInt32
from roah_rsbb_comm_ros.msg import BenchmarkState

RSBB_BENCHMARK_TOPIC = "/roah_rsbb/benchmark"
RSBB_BELL_TOPIC = "/roah_rsbb/devices/bell"
RSBB_DEVICES_STATE_TOPIC = "/roah_rsbb/devices/state"
RSBB_TABLET_CALL_TOPIC = "/roah_rsbb/tablet/call"
RSBB_TABLET_POSITION_TOPIC = "/roah_rsbb/tablet/position"
RSBB_TABLET_STATE_TOPIC = "/roah_rsbb/tablet/state"

#common topics
SBB_MESSAGES_SAVED = "/devices/messages_saved"
RSBB_BENCHMARK_STATE = "/roah_rsbb/benchmark/state"
