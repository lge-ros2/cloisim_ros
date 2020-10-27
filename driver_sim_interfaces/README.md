# driver sim interfaces

This is 'driver sim' base class.

You may need to redefine in a derived class three virtual class.

```c++
class DriverSim : public rclcpp::Node
{
public:
  explicit DriverSim(const std::string node_name, const int number_of_simbridge = 1);

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
  SimBridge* GetSimBridge(const int bridge_index = 0);
  std::string GetRobotName();
  void PublishTF();
}
```

'publisher' and 'subscriber' can be setup simultaneously at one SimBridge module.

Following bridge combination should be setup through separated SimBridge module.

- 'publisher' + ('service' or 'client')
- 'subscriber' + ('service' or 'client')
- 'service' + 'client'

Examples of using separated SimBridge modules when base class DriverSim() is inherited.

- inherit base class 'DriverSim(node name, number of sim bridge);'
- Call sim bridge using bridge index
  - GetSimBridge(0)-> ...
  - GetSimBridge(1)-> ...
  - GetSimBridge(0)->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  - GetSimBridge(1)->Connect(SimBridge::Mode::CLIENT, m_hashKeySub + "Info");
  - GetSimBridge(0)->Disconnect();
  - GetSimBridge(1)->Disconnect();
