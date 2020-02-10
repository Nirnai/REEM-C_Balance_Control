# Balance control for the humanoid platform REEM-C

The underactuated nature and the inherent instability of a biped robot, lead to the difficult control problem of maintaining balance. This repository implements different control strategies to balance the humanoid robot REEM-C from the PAL Robotics family. One major advantage of using the REEM-C platform, is it's compatibility with the ROS framework and the simulation environment Gazebo. The concept of asynchronous nodes as used in ROS makes it necessary to have a communication layer which adds overhead. This leads to lag and non real time behavior. To overcome this problem the ROS Control package will be used. 
The dynamics of the Center of Mass of a Humanoid are effected by the Zero Moment Point (ZMP) and the Center of Mass (COM). To balance a biped it is necessary to control these two quantities. 
 

## Center of Mass estimation (COM)
Since REEM-C is described in a URDF file, which shows how each link is configured relative to each other, the only step needed is to 
compute the forward kinematics for each link for a given joint configuration. There already exist libraries in C++ that make kinematic computations simple. In the scope of this research project the *Kinematics and Dynamics Library* (KDL) from Orocos will be used. It comes with ROS Kinetic and has an easy interface to load kinematic models from URDF files. The data structure that is obtained from the conversion is a tree named a `KDL::Tree`. Each node in the tree consists of a `KDL::TreeElement` which contains a `KDL::Segment` and pointers to the parent and children nodes. The segment holds all the kinematic information needed to compute the COM for that segment. Since it is a tree, a recursive algorithm is necessary to implement the estimation of the overall COM estimation.

```c++
KDL::Vector computeCOM(KDL::Tree& tree)
{
    // Initialize all variables to zero
    KDL::Frame tf = KDL::Frame::Identity();
    KDL::Vector com = KDL::Vector::Zero();
    double M = 0;
    recursiveCOM(tree.getRootSegment(), tf, com, M);
    com = com/M;
    return com;
}


void recursiveCOM(KDL::SegmentMap::const_iterator element,
                                      KDL::Frame& tf,
                                      KDL::Vector& com,
                                      double& M)
{
    double q = 0.0;

    auto& segment = element->second.segment;
    auto& children = element->second.children;

    // Check if joint is non fixed type
    if(segment.getJoint().getType() != KDL::Joint::JointType::None)
    {
        /* Get joint name from KDL model and 
           read current value from hardware interface */
        string joint_name = segment.getJoint().getName();
        q = joints_[joint_name].getPosition();
    }

    // Compute transform to current segment
    KDL::Frame curr_tf = tf * segment.pose(q);
    // Get Mass and COM of current segement
    KDL::Vector curr_com = segment.getInertia().getCOG();
    double m = segment.getInertia().getMass();

    // Add to existing COM
    M += m;
    com += m * (curr_tf * curr_com);
    for ( auto child : children )
    {
        recursiveCOM(child, curr_tf, tf_left_sole, tf_right_sole, com, M);
    }

}
```

## Zero Moment Point estimation 
To be able to estimate the zero moment point, it is necessary to measure the Ground Reaction Forces (GRF) at each foot. It is possible to do so with force-torque sensors at the feet of the REEM-C.

```c++
KDL::Vector computeZMP(KDL::Tree& model)
{
    KDL::Vector zmp = KDL::Vector::Zero();
    // Read relevant forces
    double fz_total = sensors_["left_ft_sensor"].getForce()[2] + 
                      sensors_["right_ft_sensor"].getForce()[2];
    // sensor_names as global variable 
    for(auto sensor_name : sensor_names)
    {
        // Read relevant torques from hardware interface
        KDL::Vector trq = KDL::Vector::Zero();
        std::copy(
                    sensors_[sensor_name.first].getTorque(),
                    sensors_[sensor_name.first].getTorque() + 3,
                    std::begin(trq.data)
                 );
        // Get Transform between base and current force-torque sensor
        KDL::Frame tf = getTF(model, "base_link", sensor_name.second);
        double fz = sensors_[sensor_name.first].getForce()[2];

        // Compute single support ZMP for each foot
        KDL::Vector tmp = KDL::Vector::Zero();
        tmp[0] = -trq[1]/fz;
        tmp[1] =  trq[0]/fz;

        // Transform single support ZMP to base_link
        tmp = tf * tmp;
        // Add weighted single support ZMP to double support ZMP
        zmp += tmp * fz/fz_total;
    }
    return zmp;
}
```
## Actuation
The hardware interface of REEM-C provides a position controlled interface. Thus position commands will be sent to each joint. KDL offers a simple interface for commanding individual limbs of a robot. It allows to extract sub chains from a kinematic tree and solve the inverse kinematics only for those sub chains. This improves computational speed.

```c++
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

using namespace KDL;

void commandCOM(Tree& tree, Frame& COM_des)
{   
    map<string, Chain> kinematicChains;
    // Left Leg kinematic chain
    tree.getChain("left_sole_link", "base_link", kinematicChains["left"]);
    // Right Leg kinematic chain
    tree.getChain("right_sole_link", "base_link",kinematicChains["right"]);

    for(auto chain: kinematicChains)
    {
        ChainIkSolverPos_LMA iksolver = ChainIkSolverPos_LMA(chain.second);
        JntArray q = getChainState(chain.second);
        JntArray q_des;
        iksolver.CartToJnt(q,COM_des,q_des);
        setChainState(chain.second, q_des);
    }
}
```

## Controllers and Results
The scenario that the controllers are exposed to is a force of 40 newton applied in the positive x direction for 1s. COM (x) and ZMP(p) data will be recorded for 20s and the force will be applied at 10s. To prevent REEM-C to come to close to a singularity a crouching position is obtained before the start of the recording.
### P-Control
The first controller implemented is a proportional feedback control composed of two error terms. 


$$
u = \dot{c} = - k_1 (c - p) - k_2 (c - c_{ref})
$$


Firstly the difference between the COM and the desired COM is used, to drive the COM to the desired COM. Secondly, the difference between COM and ZMP is used to drive the COM towards the ZMP. These second error term is used to achieve a compliant behavior.

![P-Control Result](/media/pcontrol-1.png)

### LQR-Control
Due to the existence of a model, it is also possible to use optimal control. Especially for a linear model, a closed form optimal control solution already exists, namely the Linear Quadratic Regulator (LQR). LQR finds the optimal gains for a state feedback law, to drive the state to it's equilibrium. We are interested in stabilizing the ZMP and COM. Both are influenced by the COM acceleration, which will be used as control input u.
This leads to the following state space model:

$$\begin{bmatrix}
        \dot{p} \\ \dot{x} \\ \ddot{x}
    \end{bmatrix} = 
    \begin{bmatrix}
        -1/T & 1/T & 0 \\
          0  &  0  & 1 \\
          0  &  0  & 0
    \end{bmatrix} 
    \begin{bmatrix}
        p \\ x \\ \dot{x}
    \end{bmatrix} + 
    \begin{bmatrix}
        - z_x/(gT) \\ 0 \\ 1
    \end{bmatrix} u$$

The LQR gains are then used to feedback the state as follows:

$$u = - \mathbf{K} \vec{x} = -k_1 p -k_2 c - k_3 \dot{c}$$

![P-Control Result](/media/lqrcontrol-1.png)

### CP-Control
All of the previous controllers were solving a regulation task. But the problem of balancing, can also be formulated as a tracking task. For that a reference trajectory for the ZMP is necessary. The Capture Point (CP) is a good candidate for that. The Capture Point is the point, on which the robot needs to step on to, to come to a complete rest. In conclusion, if it is possible to keep the CP insides the support base, or even at the center of the support base, the robots dynamics will come to a rest. Its computation is defined as follows:

$$\xi = x + \sqrt{\frac{z_x}{g}}\dot{x}$$

The first order derivative of the CP shows that it is pushed away by the ZMP.

$$\dot{\xi} = \frac{g}{z_x}(\xi - p)$$

A proportional controller will try to track a reference ZMP $p_{ref} = \xi - K_p(\xi_{des} - \xi)$ and thereby drive the capture point to 0. 

$$
u = \dot{x} = -k_p (p - p_{ref})
$$

![P-Control Result](/media/cpcontrol-1.png)