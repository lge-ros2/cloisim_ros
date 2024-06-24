# CLOiSim-ROS

This is 'CLOiSim-ROS' base class.

You may need to redefine in a derived class three virtual class.

```c++
namespace cloisim_ros
{
  class Base : public rclcpp::Node
  {
  public:
    explicit Base(const std::string node_name);
    explicit Base(const std::string node_name, const rclcpp::NodeOptions &options);
    explicit Base(const std::string node_name, const std::string namespace_);
    explicit Base(const std::string node_name, const std::string namespace_, const rclcpp::NodeOptions &options);
    virtual ~Base();


  protected:
    virtual void Initialize() = 0;
    virtual void Deinitialize() = 0;

    void Start();
    void Stop();

    bool IsRunThread();
    rclcpp::Node* GetNode();
    std::string GetRobotName();
    zmq::Bridge* CreateBridge();
  }
  }
```

## parameter

TF/TFStatic is enabled to publish by default.

If you want disable publishing TF and TF Static, just set the parameter `enable_tf:=False`.

Some of cloisim_ros packages are not allowed to disable.

- Actor, Elevator_System, GroundTruth, World
