#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.h>

#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <stack>
#include <deque>
#include <cmath>

#include <Eigen/Dense>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "utils.hpp"

using namespace hardware_interface;
using namespace filter;

using std::string;
using std::map;
using std::vector;
using std::stack;
using std::deque;
using std::asin;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace reemc_balance_control{


  class ZMPController: public controller_interface::MultiInterfaceController<PositionJointInterface, ForceTorqueSensorInterface>
  {
  public:
    ZMPController();
    // Initializers
    bool init( RobotHW*                   robot_hw,
              ros::NodeHandle&            controller_nh);
    bool init_sensors(RobotHW* robot_hw, ros::NodeHandle& controller_nh);
    bool init_joints(RobotHW* robot_hw, ros::NodeHandle& controller_nh);
    bool goToHomePosition(double start_time);

    // ROS-Control Functions
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

    KDL::Frame setWorldFrame(KDL::Tree& kdlModel, urdf::Model& urdfModel);
    KDL::Vector computeCOM(KDL::Tree& model);
    KDL::Vector computeZMP(KDL::Tree& model);
    KDL::Vector computeCP(KDL::Vector& com);
    KDL::JntArray getChainState(KDL::Chain& chain);
    KDL::JntArray getChainCommand(KDL::Chain& chain);
    void setChainState(KDL::Chain& chain, KDL::JntArray& q);
    void commandComPos(KDL::Tree& model, double x);

    double p_controller(KDL::Vector com, KDL::Vector zmp);
    double lqr_acc_controller(KDL::Vector com, KDL::Vector zmp, double xd);
    double lqr_jerk_controller(double x, double v, double a);
    double lqg_acc_controller(KDL::Vector com, KDL::Vector zmp);
    double cp_controller(KDL::Vector zmp, KDL::Vector zmp_ref);



    // Helper Functions
    KDL::Frame getTF(KDL::Tree& model, string startlink, string endlink);
    void recursiveCOM(KDL::SegmentMap::const_iterator segment,
                      KDL::Frame& tf,
                      KDL::Vector& com,
                      double& M);
    void writeToCSV(std::string path, std::vector<VectorXd>& data);
    int recordData(string filename,int start, int samples, VectorXd& dataSample);
    void visualize_KDLVector(KDL::Vector v);
    void visualize_com();
    void visualize_zmp();
    void visualize_cp();
    void apply_froce(double force);

  private:
    // Joints
    const string kinematicChainNames[5] = { "head", "left_arm", "right_arm", "left_leg", "right_leg" };
    map<string, JointHandle> joints_;
    map<string, KDL::Chain> kinematicChains;

    // Sensors
    /* const string sensor_names[2] = {"left_ft_sensor", "right_ft_sensor"}; */
    map<string, string> sensor_names = {{"left_ft_sensor", "left_sole_link"},{"right_ft_sensor", "right_sole_link"}};
    map<string, ForceTorqueSensorHandle> sensors_;

    // Eigen output formats
    Eigen::IOFormat csv;

    // Robot Model
    std::unique_ptr<urdf::Model> URDFmodel;
    std::unique_ptr<KDL::Tree> KDLmodel;

    // Robot Parameters
    double M;
    KDL::Vector com;
    KDL::Vector zmp;
    KDL::Vector cp;

    // World Frame
    KDL::Frame world;



    // Debugging Variables
    visualization_msgs::Marker viz_msg;
    geometry_msgs::PolygonStamped supportBase;
    ros::Publisher v_pub;
    ros::Publisher sb_pub;
    ros::Publisher com_pub;
    ros::Publisher zmp_pub;
    ros::Publisher cp_pub;
    ros::ServiceClient applyForceClient;


  };
  PLUGINLIB_EXPORT_CLASS(reemc_balance_control::ZMPController, controller_interface::ControllerBase);
}
