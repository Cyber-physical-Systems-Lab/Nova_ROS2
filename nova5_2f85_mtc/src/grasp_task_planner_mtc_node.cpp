#include <memory>
#include <string>
#include <map>
#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace mtc = moveit::task_constructor;

namespace
{

struct PoseComponents
{
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

geometry_msgs::msg::PoseStamped toPoseStamped(
  const PoseComponents& p,
  const std::string& frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.position.z = p.z;
  pose.pose.orientation.x = p.qx;
  pose.pose.orientation.y = p.qy;
  pose.pose.orientation.z = p.qz;
  pose.pose.orientation.w = p.qw;
  return pose;
}

std::string poseToString(const geometry_msgs::msg::PoseStamped& pose)
{
  char buf[256];
  std::snprintf(
    buf, sizeof(buf),
    "frame=%s x=%.3f y=%.3f z=%.3f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
    pose.header.frame_id.c_str(),
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);
  return std::string(buf);
}

geometry_msgs::msg::PoseStamped offsetWorldZ(
  const geometry_msgs::msg::PoseStamped& pose,
  double dz)
{
  auto out = pose;
  out.pose.position.z += dz;
  return out;
}

}  // namespace

class GraspTaskPlannerMTC : public rclcpp::Node
{
public:
  GraspTaskPlannerMTC()
  : Node("grasp_task_planner_mtc"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("command_topic", "/web_command");
    declare_parameter<std::string>("object_id_topic", "/grasp_object_id");

    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("ik_frame", "tool_link");

    declare_parameter<std::string>("arm_group", "nova5_group");
    declare_parameter<std::string>("gripper_group", "gripper");

    declare_parameter<std::string>("open_named_target", "open");
    declare_parameter<std::string>("close_named_target", "close");

    declare_parameter<double>("pre_grasp_offset_m", 0.08);
    declare_parameter<double>("lift_height_m", 0.08);

    declare_parameter<double>("cartesian_step_m", 0.005);
    declare_parameter<double>("cartesian_velocity_scaling", 1.0);
    declare_parameter<double>("cartesian_acceleration_scaling", 1.0);

    declare_parameter<double>("connect_timeout_s", 5.0);
    declare_parameter<int>("num_planning_solutions", 1);

    command_topic_ = get_parameter("command_topic").as_string();
    object_id_topic_ = get_parameter("object_id_topic").as_string();

    base_frame_ = get_parameter("base_frame").as_string();
    ik_frame_ = get_parameter("ik_frame").as_string();
    arm_group_ = get_parameter("arm_group").as_string();
    gripper_group_ = get_parameter("gripper_group").as_string();

    open_named_target_ = get_parameter("open_named_target").as_string();
    close_named_target_ = get_parameter("close_named_target").as_string();

    pre_grasp_offset_m_ = get_parameter("pre_grasp_offset_m").as_double();
    lift_height_m_ = get_parameter("lift_height_m").as_double();
    cartesian_step_m_ = get_parameter("cartesian_step_m").as_double();
    cartesian_velocity_scaling_ = get_parameter("cartesian_velocity_scaling").as_double();
    cartesian_acceleration_scaling_ = get_parameter("cartesian_acceleration_scaling").as_double();
    connect_timeout_s_ = get_parameter("connect_timeout_s").as_double();
    num_planning_solutions_ = get_parameter("num_planning_solutions").as_int();

    drop_pose_ = toPoseStamped(
      PoseComponents{-0.5, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0},
      "base_link");

    home_pose_ = toPoseStamped(
      PoseComponents{0.04, 0.3, 0.3, 1.0, 0.0, 0.0, 0.0},
      "base_link");

    grasp_poses_.emplace(
      1, toPoseStamped(PoseComponents{-0.25, 0.41, 0.23, 1.0, 0.0, 0.0, 0.0}, base_frame_));
    grasp_poses_.emplace(
      2, toPoseStamped(PoseComponents{-0.087, 0.41, 0.23, 1.0, 0.0, 0.0, 0.0}, base_frame_));
    grasp_poses_.emplace(
      3, toPoseStamped(PoseComponents{0.064, 0.466, 0.23, 1.0, 0.0, 0.0, 0.0}, base_frame_));
    grasp_poses_.emplace(
      4, toPoseStamped(PoseComponents{-0.046, 0.572, 0.23, 1.0, 0.0, 0.0, 0.0}, base_frame_));
    grasp_poses_.emplace(
      5, toPoseStamped(PoseComponents{0.115, 0.616, 0.23, 1.0, 0.0, 0.0, 0.0}, base_frame_));

    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic_, 10,
      std::bind(&GraspTaskPlannerMTC::commandCb, this, std::placeholders::_1));

    object_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      object_id_topic_, 10,
      std::bind(&GraspTaskPlannerMTC::objectIdCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GraspTaskPlannerMTC ready");
    RCLCPP_INFO(get_logger(), "command_topic=%s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "object_id_topic=%s", object_id_topic_.c_str());
    RCLCPP_INFO(get_logger(), "arm_group=%s gripper_group=%s ik_frame=%s",
                arm_group_.c_str(), gripper_group_.c_str(), ik_frame_.c_str());
  }

private:
  void objectIdCb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int object_id = static_cast<int>(msg->data);
    if (grasp_poses_.find(object_id) == grasp_poses_.end()) {
      RCLCPP_WARN(get_logger(), "Ignoring unsupported object id %d, expected 1..5", object_id);
      return;
    }

    selected_object_id_ = object_id;
    RCLCPP_INFO(get_logger(), "Selected grasp object id %d", object_id);
  }

  void commandCb(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string cmd = msg->data;

    if (busy_) {
      RCLCPP_WARN(get_logger(), "Ignoring command '%s' because task execution is in progress", cmd.c_str());
      return;
    }

    if (cmd == "start_grasp") {
      if (!selected_object_id_) {
        RCLCPP_WARN(get_logger(), "Ignoring start_grasp because no object id has been received yet");
        return;
      }
      runTask(buildStartGraspTask(*selected_object_id_), "start_grasp");
      return;
    }

    if (cmd == "stop_grasp") {
      runTask(buildStopGraspTask(), "stop_grasp");
      return;
    }

    if (cmd == "go_home") {
      runTask(buildGoHomeTask(), "go_home");
      return;
    }

    RCLCPP_WARN(get_logger(), "Ignoring unknown web command '%s'", cmd.c_str());
  }

  mtc::Task buildBaseTask(const std::string& task_name)
  {
    mtc::Task task;
    task.stages()->setName(task_name);
    task.loadRobotModel(shared_from_this());

    task.setProperty("group", arm_group_);
    task.setProperty("eef", gripper_group_);
    task.setProperty("ik_frame", ik_frame_);
    return task;
  }

  std::shared_ptr<mtc::solvers::PipelinePlanner> makePipelinePlanner()
  {
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    return planner;
  }

  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> makeJointInterpolationPlanner()
  {
    return std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  }

  std::shared_ptr<mtc::solvers::CartesianPath> makeCartesianPlanner()
  {
    auto planner = std::make_shared<mtc::solvers::CartesianPath>();
    planner->setStepSize(cartesian_step_m_);
    planner->setMaxVelocityScalingFactor(cartesian_velocity_scaling_);
    planner->setMaxAccelerationScalingFactor(cartesian_acceleration_scaling_);
    return planner;
  }

  mtc::Task buildStartGraspTask(int object_id)
  {
    auto task = buildBaseTask("start_grasp_task");

    auto pipeline = makePipelinePlanner();
    auto joint_interp = makeJointInterpolationPlanner();
    auto cartesian = makeCartesianPlanner();

    const auto grasp_pose = grasp_poses_.at(object_id);
    const auto pre_grasp_pose = offsetWorldZ(grasp_pose, pre_grasp_offset_m_);

    RCLCPP_INFO(get_logger(), "Building start_grasp task for object %d", object_id);
    RCLCPP_INFO(get_logger(), "pre_grasp: %s", poseToString(pre_grasp_pose).c_str());
    RCLCPP_INFO(get_logger(), "grasp: %s", poseToString(grasp_pose).c_str());

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(gripper_group_);
      stage->setGoal(open_named_target_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", pipeline);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(arm_group_);
      stage->setGoal(pre_grasp_pose);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "cartesian descend to grasp", cartesian);
      stage->restrictDirection(mtc::stages::MoveRelative::FORWARD);
      stage->properties().set("group", arm_group_);
      stage->setIKFrame(ik_frame_);
      stage->setMinMaxDistance(pre_grasp_offset_m_, pre_grasp_offset_m_);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = base_frame_;
      direction.vector.z = -1.0;
      stage->setDirection(direction);

      task.add(std::move(stage));
    }

    return task;
  }

  mtc::Task buildStopGraspTask()
  {
    auto task = buildBaseTask("stop_grasp_task");

    auto pipeline = makePipelinePlanner();
    auto joint_interp = makeJointInterpolationPlanner();
    auto cartesian = makeCartesianPlanner();

    RCLCPP_INFO(get_logger(), "Building stop_grasp task");

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_interp);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(gripper_group_);
      stage->setGoal(close_named_target_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
      stage->restrictDirection(mtc::stages::MoveRelative::FORWARD);
      stage->properties().set("group", arm_group_);
      stage->setIKFrame(ik_frame_);
      stage->setMinMaxDistance(lift_height_m_, lift_height_m_);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = base_frame_;
      direction.vector.z = 1.0;
      stage->setDirection(direction);

      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to drop pose", pipeline);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(arm_group_);
      stage->setGoal(drop_pose_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper at drop", joint_interp);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(gripper_group_);
      stage->setGoal(open_named_target_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move home", pipeline);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(arm_group_);
      stage->setGoal(home_pose_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper at home", joint_interp);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(gripper_group_);
      stage->setGoal(open_named_target_);
      task.add(std::move(stage));
    }

    return task;
  }

  mtc::Task buildGoHomeTask()
  {
    auto task = buildBaseTask("go_home_task");

    auto pipeline = makePipelinePlanner();
    auto joint_interp = makeJointInterpolationPlanner();

    RCLCPP_INFO(get_logger(), "Building go_home task");

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move home", pipeline);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(arm_group_);
      stage->setGoal(home_pose_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp);
      stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
      stage->setGroup(gripper_group_);
      stage->setGoal(open_named_target_);
      task.add(std::move(stage));
    }

    return task;
  }

  void runTask(mtc::Task task, const std::string& label)
  {
    busy_ = true;

    try {
      task.init();
    } catch (const mtc::InitStageException& e) {
      busy_ = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Task '" << label << "' init failed:\n" << e);
      return;
    } catch (const std::exception& e) {
      busy_ = false;
      RCLCPP_ERROR(get_logger(), "Task '%s' init exception: %s", label.c_str(), e.what());
      return;
    }

    if (!task.plan(num_planning_solutions_)) {
      busy_ = false;
      RCLCPP_ERROR(get_logger(), "Task '%s' planning failed", label.c_str());
      return;
    }

    if (task.solutions().empty()) {
      busy_ = false;
      RCLCPP_ERROR(get_logger(), "Task '%s' produced no solutions", label.c_str());
      return;
    }

    const auto result = task.execute(*task.solutions().front());
    busy_ = false;

    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "Task '%s' execution failed with MoveIt code %d",
        label.c_str(),
        result.val);
      return;
    }

    RCLCPP_INFO(get_logger(), "Task '%s' executed successfully", label.c_str());
  }

private:
  std::string command_topic_;
  std::string object_id_topic_;

  std::string base_frame_;
  std::string ik_frame_;
  std::string arm_group_;
  std::string gripper_group_;

  std::string open_named_target_;
  std::string close_named_target_;

  double pre_grasp_offset_m_{0.08};
  double lift_height_m_{0.08};
  double cartesian_step_m_{0.005};
  double cartesian_velocity_scaling_{1.0};
  double cartesian_acceleration_scaling_{1.0};
  double connect_timeout_s_{5.0};
  int num_planning_solutions_{1};

  bool busy_{false};
  std::optional<int> selected_object_id_;

  geometry_msgs::msg::PoseStamped drop_pose_;
  geometry_msgs::msg::PoseStamped home_pose_;
  std::map<int, geometry_msgs::msg::PoseStamped> grasp_poses_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr object_id_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspTaskPlannerMTC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
