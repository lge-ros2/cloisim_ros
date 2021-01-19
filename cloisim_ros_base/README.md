# CLOiSim-ROS

This is 'CLOiSim-ROS' base class.

You may need to redefine in a derived class three virtual class.

```c++
namespace cloisim_ros
{
  class Base : public rclcpp::Node
  {
  public:
    explicit Base(const std::string node_name_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const rclcpp::NodeOptions &options_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const std::string namespace_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const std::string namespace_, const rclcpp::NodeOptions &options_, const int number_of_bridges = 1);


  protected:
    virtual void Initialize() = 0;
    virtual void Deinitialize() = 0;
    virtual void UpdateData() = 0;

    void Start();
    void Stop();

    void AddTf2(const geometry_msgs::msg::TransformStamped _tf);
    void AddStaticTf2(const geometry_msgs::msg::TransformStamped _tf);

    bool IsRunThread();
    rclcpp::Node* GetNode();
    zmq::Bridge* GetBridge(const int bridge_index = 0);
    std::string GetRobotName();
    void PublishTF();
  }
  }
```

'publisher' and 'subscriber' can be setup simultaneously at one Bridge module.

Following bridge combination should be setup through separated Bridge module.

- 'publisher' + ('service' or 'client')
- 'subscriber' + ('service' or 'client')
- 'service' + 'client'

Examples of using separated Bridge modules when base class Base() is inherited.

- inherit base class 'Base(node name, number of sim bridge);'
- Call sim bridge using bridge index
  - GetBridge(0)-> ...
  - GetBridge(1)-> ...
  - GetBridge(0)->Connect(zmq::Bridge::Mode::SUB, hashKeySub_);
  - GetBridge(1)->Connect(zmq::Bridge::Mode::CLIENT, hashKeySub_ + "Info");
  - GetBridge(0)->Disconnect();
  - GetBridge(1)->Disconnect();
