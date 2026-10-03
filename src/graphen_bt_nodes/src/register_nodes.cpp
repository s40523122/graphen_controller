#include "behaviortree_cpp_v3/bt_factory.h"
#include "graphen_bt_nodes/execution_rate_logger.hpp"
#include "graphen_bt_nodes/high_freq_control.hpp"
#include "graphen_bt_nodes/is_ok.hpp"
#include "graphen_bt_nodes/rate_control.hpp"
#include "graphen_bt_nodes/say_hello.hpp"
#include "graphen_bt_nodes/sys_count_nodes.hpp"
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
}
