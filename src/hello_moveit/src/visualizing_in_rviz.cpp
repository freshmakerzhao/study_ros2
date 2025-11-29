#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  // ==========================================
  // 1. 初始化 ROS 2 节点
  // ==========================================
  rclcpp::init(argc, argv);
  
  // 创建节点选项，开启 "自动声明参数" 功能。
  // 如果不加这个 true，MoveIt 可能会报错找不到参数。
  auto const node = std::make_shared<rclcpp::Node>(
      "visualizing_in_rviz",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个日志记录器，方便在终端打印 [INFO] 信息
  auto const logger = rclcpp::get_logger("visualizing_in_rviz");

  // ==========================================
  // 2. 启动后台线程
  // ==========================================
  // MoveGroupInterface 需要实时知道机器人的当前状态（关节角度、坐标变换 TF）。
  // 这需要节点不断处理后台消息 (spin)。
  // 开启一个新的线程专门用来 spin 节点，防止主线程被阻塞。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // 使用 Lambda 表达式启动线程
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // ==========================================
  // 3. 创建 MoveIt 核心接口
  // ==========================================
  // "panda_arm" 是我们在 SRDF 配置文件中定义的规划组名称
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // ==========================================
  // 4. 初始化可视化工具 (Visual Tools)
  // ==========================================
  // 这个工具负责在 RViz 里画线、写字、添加按钮。
  // "panda_link0" 是参考坐标系（基座），所有的可视化都在这个坐标系下进行。
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, 
      "panda_link0", 
      rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()
  };
  
  // 清空 RViz 里之前可能残留的标记
  moveit_visual_tools.deleteAllMarkers();
  // 加载 "Next" 按钮面板 (Remote Control)
  moveit_visual_tools.loadRemoteControl();

  // ==========================================
  // 5. 定义几个辅助函数
  // ==========================================
  // 让 main 函数的主逻辑更干净。
  
  // 辅助函数 draw_title：在机器人头顶写标题 (Text)
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // 把字写在基座上方 1 米处
      return msg;
    }();
    // 发布文字，颜色白色，字号 XLARGE
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  // 辅助函数 prompt：阻塞程序，等待你在 RViz 点 "Next" 按钮
  auto const prompt = [&moveit_visual_tools](auto text) {
      moveit_visual_tools.prompt(text);
  };

  // 辅助函数 draw_trajectory_tool_path：画出末端执行器的轨迹线 (Path)
  // 这里捕获了 jmg (JointModelGroup)，即机械臂的模型信息
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
          auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // ==========================================
  // 6. 设置运动目标
  // ==========================================
  // 定义一个具体的空间位置 (Pose: 位置 x,y,z + 姿态四元数 w,x,y,z)
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0; // 这是一个无旋转的初始姿态
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  
  // 设置目标位置 target_pose
  move_group_interface.setPoseTarget(target_pose);

  // ==========================================
  // 7. 规划 (Planning)
  // ==========================================
  // 第一步：暂停，等 next
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning"); // 更改头顶文字
  moveit_visual_tools.trigger(); // 必须调用 trigger，刚才设置的文字才会发送给 RViz 显示出来
  
  // 执行规划计算
  // plan() 函数只计算路径，机器人的真身是不会动的
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // ==========================================
  // 8. 执行 (Execute)
  // ==========================================
  if (success) {
    // 如果规划成功，先把规划出的轨迹画成线显示出来
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger(); // 再次提交显示请求
    
    // 第二步：暂停，确认路径无误后，等 next 按钮才真正动
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    
    // 真正执行规划的路径
    move_group_interface.execute(plan);
  } else {
    // 如果规划失败（比如目标太远够不着，或者有障碍物）
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // ==========================================
  // 9. 结束清理
  // ==========================================
  rclcpp::shutdown(); // 关闭 ROS 系统
  spinner.join();     // 等待后台线程安全结束
  return 0;
}