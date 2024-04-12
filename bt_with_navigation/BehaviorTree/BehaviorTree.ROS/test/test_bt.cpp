#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>

using namespace BT;

// 定义一个同步行为节点，用于打印一个数字
class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    int value = 0;
    if( getInput("message", value ) ){
      std::cout << "PrintValue: " << value << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<int>("message") };
  }
};

// 定义一个ROS服务行为节点，用于调用AddTwoInts服务
class AddTwoIntsAction: public RosServiceNode<behaviortree_ros::AddTwoInts>
{
public:
  AddTwoIntsAction( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::AddTwoInts>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<int>("first_int"),
      InputPort<int>("second_int"),
      OutputPort<int>("sum") };
  }

  void sendRequest(RequestType& request) override
  {
    getInput("first_int", request.a);
    getInput("second_int", request.b);
    expected_result_ = request.a + request.b;
    ROS_INFO("AddTwoInts: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("AddTwoInts: response received");
    if( rep.sum == expected_result_)
    {
      setOutput<int>("sum", rep.sum);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("AddTwoInts replied something unexpected: %d", rep.sum);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("AddTwoInts request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  int expected_result_;
};

// 定义一个ROS行为节点，用于执行FibonacciAction
class FibonacciServer: public RosActionNode<behaviortree_ros::FibonacciAction>
{
public:
  FibonacciServer( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<behaviortree_ros::FibonacciAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<int>("order"),
      OutputPort<int>("result") };
  }

  bool sendGoal(GoalType& goal) override
  {
    if( !getInput<int>("order", goal.order) )
    {
      // 终止整个行为。返回FAILURE状态
      return false;
    }
    expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // 假设order为5
    ROS_INFO("FibonacciAction: sending request");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("FibonacciAction: result received");
    int fibonacci_result = 0;
    for( int n: res.sequence)
    {
      fibonacci_result += n;
    }
    if( fibonacci_result == expected_result_)
    {
      setOutput<int>("result", fibonacci_result);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("FibonacciAction replied something unexpected: %d", fibonacci_result);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("FibonacciAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("FibonacciAction halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
};

// 定义行为树XML文本
static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <AddTwoInts service_name = "add_two_ints"
                        first_int = "3" second_int = "4"
                        sum = "{add_two_result}" />
            <PrintValue message="{add_two_result}"/>

            <RetryUntilSuccessful num_attempts="4">
                <Timeout msec="300">
                    <Fibonacci server_name="fibonacci" order="5"
                               result="{fibonacci_result}" />
                </Timeout>
            </RetryUntilSuccessful>
            <PrintValue message="{fibonacci_result}"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;

  // 创建行为树工厂
  BehaviorTreeFactory factory;

  // 注册行为树节点类型
  factory.registerNodeType<PrintValue>("PrintValue");
  RegisterRosService<AddTwoIntsAction>(factory, "AddTwoInts", nh);
  RegisterRosAction<FibonacciServer>(factory, "Fibonacci", nh);

  // 从XML文本创建行为树
  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout <<"status:  " <<status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}
