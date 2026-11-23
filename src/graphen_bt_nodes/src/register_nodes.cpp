#include "behaviortree_cpp_v3/bt_factory.h"
#include "graphen_bt_nodes/compare_nodes.hpp"
#include "graphen_bt_nodes/execution_rate_logger.hpp"
#include "graphen_bt_nodes/get_pixel_error.hpp"
#include "graphen_bt_nodes/get_state_value.hpp"
#include "graphen_bt_nodes/high_freq_control.hpp"
#include "graphen_bt_nodes/is_ok.hpp"
#include "graphen_bt_nodes/joint_velocity.hpp"
#include "graphen_bt_nodes/playback_nodes.hpp"
#include "graphen_bt_nodes/rate_control.hpp"
#include "graphen_bt_nodes/say_hello.hpp"
#include "graphen_bt_nodes/send_on_change.hpp"
#include "graphen_bt_nodes/sys_count_nodes.hpp"
#include "graphen_bt_nodes/value_nodes.hpp"
#include "graphen_bt_nodes/wait.hpp"

// 以 factory.registerFromPlugin() 載入本函式庫時呼叫，註冊所有自訂節點
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<graphen::bt::HighFreqControlNode>("HighFreqControlNode");
  factory.registerNodeType<graphen::bt::RateControl>("RateControl");
  factory.registerNodeType<graphen::bt::ExecutionRateLogger>("ExecutionRateLogger");
  factory.registerNodeType<graphen::bt::SayHello>("SayHello");
  factory.registerNodeType<graphen::bt::Wait>("Wait");
  factory.registerNodeType<graphen::bt::IsOk>("IsOk");
  factory.registerNodeType<graphen::bt::IncrementSysCount>("IncrementSysCount");
  factory.registerNodeType<graphen::bt::CheckSysCount>("CheckSysCount");
  factory.registerNodeType<graphen::bt::ClearSysCount>("ClearSysCount");
  factory.registerNodeType<graphen::bt::GetStateValue>("GetStateValue");
  factory.registerNodeType<graphen::bt::IsBelow>("IsBelow");
  factory.registerNodeType<graphen::bt::IsAbove>("IsAbove");
  // 播放類：一種類型一個節點（共同的行為在 PlaybackActionNode）
  factory.registerNodeType<graphen::bt::SwitchMode>("SwitchMode");
  factory.registerNodeType<graphen::bt::PlayChoreo>("PlayChoreo");
  factory.registerNodeType<graphen::bt::PlayBvh>("PlayBvh");
  factory.registerNodeType<graphen::bt::RunPolicy>("RunPolicy");
  // 低頻速度命令與視覺跟隨（face_track.xml）
  factory.registerNodeType<graphen::bt::JointVelocity>("JointVelocity");
  factory.registerNodeType<graphen::bt::GetPixelError>("GetPixelError");
  factory.registerNodeType<graphen::bt::SendOnChange>("SendOnChange");
  factory.registerNodeType<graphen::bt::Multiply>("Multiply");
  factory.registerNodeType<graphen::bt::JoinValues>("JoinValues");
}
