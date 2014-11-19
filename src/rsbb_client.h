#ifndef COMM_CLIENT_H
#define COMM_CLIENT_H

#include <roah_rsbb_comm_ros/Benchmark.h>
#include <std_msgs/Empty.h>
#include <roah_rsbb_comm_ros/DevicesState.h>
#include <geometry_msgs/Pose2D.h>
#include <roah_rsbb_comm_ros/TabletState.h>
#include <std_msgs/UInt32.h>
#include <roah_rsbb_comm_ros/BenchmarkState.h>

#define RSBB_BENCHMARK_TOPIC "/roah_rsbb/benchmark"
#define RSBB_BELL_TOPIC "/roah_rsbb/devices/bell""
#define RSBB_DEVICES_STATE_TOPIC "/roah_rsbb/devices/state"
#define RSBB_TABLET_CALL_TOPIC "/roah_rsbb/tablet/call"
#define RSBB_TABLET_POSITION_TOPIC "/roah_rsbb/tablet/position"
#define RSBB_TABLET_STATE_TOPIC "/roah_rsbb/tablet/state"

#define RSBB_MESSAGES_SAVED "/devices/messages_saved"
#define RSBB_BENCHMARK_STATE "/roah_rsbb/benchmark/state"

#endif
