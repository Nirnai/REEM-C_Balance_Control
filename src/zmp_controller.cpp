#include "zmp_controller.hpp"


namespace reemc_balance_control
{
    ZMPController::ZMPController()
    :csv(
        Eigen::FullPrecision,
        Eigen::DontAlignCols,
        ",",
        ",",
        "",
        "",
        "",
        "\n" ),
     URDFmodel(std::make_unique<urdf::Model>()),
     KDLmodel(std::make_unique<KDL::Tree>())
    {

        std::string Fname = "/home/catkin_ws/src/reemc_balance_control/urdf/reemc_full.urdf";
        ROS_INFO("Starting to load Kinematic Model!");

        // read URDF model
        if (!URDFmodel->initFile(Fname))
        {
            ROS_ERROR_STREAM("Error loading urdf file " << Fname);
        }
        if(!kdl_parser::treeFromUrdfModel(*URDFmodel, *KDLmodel))
        {
            ROS_ERROR("Error parsing URDF to KDL!");
        }
    }
    bool ZMPController::init(RobotHW* robot_hw, ros::NodeHandle& controller_nh)
    {
        v_pub   = controller_nh.advertise<visualization_msgs::Marker>("v", 10);;
        com_pub = controller_nh.advertise<visualization_msgs::Marker>("com_marker", 10);
        zmp_pub = controller_nh.advertise<visualization_msgs::Marker>("zmp_marker", 10);
        cp_pub  = controller_nh.advertise<visualization_msgs::Marker>("cp_marker", 10);
        /* sb_pub  = controller_nh.advertise<geometry_msgs::PolygonStamped>("support_base", 10); */
        applyForceClient = controller_nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

        if(!init_sensors(robot_hw, controller_nh))
        {
            ROS_INFO("Failed to initialize force torque sensors!");
            return false;
        }
        if(!init_joints(robot_hw, controller_nh))
        {
            ROS_INFO("Failed to initialize joints sensors!");
            return false;
        }

        return true;
    }
    void ZMPController::starting(const ros::Time& time){   }
    void ZMPController::stopping(const ros::Time& time){   }
    void ZMPController::update(const ros::Time& time, const ros::Duration& period)
    {
        // Go to home position
        static double start_time = ros::Time::now().toSec();
        static bool initialized = false;
        if(!initialized)
        {
            initialized = goToHomePosition(start_time);
        }
        else
        {
            world = setWorldFrame(*KDLmodel, *URDFmodel);
            visualize_KDLVector(world.Inverse().p);

            // CoM Estimation
            com = computeCOM(*KDLmodel);
            com = world * com;
            visualize_com();

            // ZMP Estimation
            zmp = computeZMP(*KDLmodel);
            zmp = world * zmp;
            visualize_zmp();

            // CP Estimation
            cp = computeCP(com);
            visualize_cp();


            /////////////////////////// P-Control ///////////////////////////////

            /* static numeric::integral<double> ix(0.001, 0);
            double xd = p_controller(com, zmp);
            double x = ix(xd);
            commandComPos(x); */


            /////////////////////////// LQR-Control ///////////////////////////////
            /* static double a = 0;
            static double v = 0;
            static double x = 0;

            // Acc
            static numeric::integral<double> iv(0.001, 0);
            static numeric::integral<double> ia(0.001,0);
            a = lqr_acc_controller(com, zmp, v);
            v = ia(a);
            x = iv(v);
            commandComPos(x); */




           /////////////////////////// Jerk-Control ///////////////////////////////
            /* double a = 0;
            double v = 0;
            double x = 0;
            static numeric::integral<double> iv(0.001,0);
            static numeric::integral<double> ia(0.001,0);
            static numeric::integral<double> ij(0.001,0);
            double j = lqr_jerk_controller(x_hat1[0], x_hat1[1], x_hat1[2]);
            a = ij(j);
            v = ia(a);
            x = iv(v);
            commandComPos(x);*/

            /////////////////////////// ZMP-tracking-Control ///////////////////////////////
            static numeric::integral<double> ix(0.001, 0);
            KDL::Vector zmp_ref = cp + 0.75*cp;
            double v = cp_controller(zmp, zmp_ref);
            double x = ix(v);
            commandComPos(*KDLmodel, x);

            ///////////////////////////// Record Data ///////////////////////////////
            VectorXd state(3);
            state << zmp[0],com[0],zmp_ref[0];
            int sample = recordData("report_data/CP_PDControl.csv", 0, 20000, state);
            ROS_INFO_STREAM("sample = " << sample);

            if(sample == 10000)
            {
                apply_froce(40);
            }

        }
    }
    bool ZMPController::init_sensors(RobotHW* robot_hw, ros::NodeHandle& controller_nh)
    {
        ForceTorqueSensorInterface*   ft = robot_hw->get<ForceTorqueSensorInterface>();

        for(auto name: sensor_names)
        {
            std::string sensor_name;
            if (!controller_nh.getParam(name.first, sensor_name))
            {
                ROS_ERROR_STREAM("No "<<name.first<<" given (namespace:" << controller_nh.getNamespace() << ").");
                return false;
            }
            try
            {
                sensors_[name.first] = ft->getHandle(sensor_name);

                ROS_DEBUG_STREAM("Found force-torque sensor '" << sensor_name << "' in '" <<
                            internal::demangledTypeName(*ft) << "'");
            }
            catch (...)
            {
                ROS_ERROR_STREAM("Could not find force-torque sensor '" << sensor_name << "' in '" <<
                            internal::demangledTypeName(*ft) << "'");
                return false;
            }
        }

        return true;
    }
    bool ZMPController::init_joints(RobotHW* robot_hw, ros::NodeHandle& controller_nh)
    {
        PositionJointInterface* j = robot_hw->get<PositionJointInterface>();

        stack<KDL::SegmentMap::const_iterator> segments;
        segments.push(KDLmodel->getRootSegment());
        KDL::SegmentMap::const_iterator curr;

        while( segments.size() > 0 )
        {
            curr = segments.top();
            segments.pop();
            for (auto child: curr->second.children)
            {
                segments.push(child);
            }

            if( curr->second.segment.getJoint().getType() != KDL::Joint::JointType::None )
            {
                string joint_name = curr->second.segment.getJoint().getName();
                try
                {
                    JointHandle tmp = j->getHandle(joint_name);
                    joints_[joint_name] = tmp;
                    ROS_INFO_STREAM("Found joint '" << tmp.getName() << "' with position: " << tmp.getPosition());
                }
                catch (...)
                {
                    ROS_ERROR_STREAM("Could not find joint '" << joint_name);
                    return false;
                }
            }
        }
        return true;
    }
    bool ZMPController::goToHomePosition(double start_time)
    {
        KDLmodel->getChain("left_sole_link", "base_link", kinematicChains["left"]);
        KDLmodel->getChain("right_sole_link", "base_link", kinematicChains["right"]);

        KDL::JntArray start(6);
        start.data = VectorXd::Zero(6);

        KDL::JntArray current = getChainCommand(kinematicChains["left"]);

        KDL::JntArray home(6);
        home.data << 0.0, -0.2848, 0.5786, -0.2848 , 0.0, 0.0;

        VectorXd diff = current.data - home.data;
        diff = diff.cwiseAbs();
        double err = diff.sum();
        //ROS_INFO_STREAM("error to home = " << err );
        if(err >= 0.0001)
        {
            double curr_time = ros::Time::now().toSec();
            double mu = (curr_time - start_time);

            static spline<Eigen::VectorXd> interpolate(start.data, home.data);
            KDL::JntArray command;
            command.data = interpolate(mu);

            setChainState(kinematicChains["left"], command);
            setChainState(kinematicChains["right"], command);
            return false;
        }
        else
        {
            ROS_INFO("Reached Home!");
            return true;
        }

    }

    KDL::Frame ZMPController::setWorldFrame(KDL::Tree& kdlModel, urdf::Model& urdfModel)
    {
        // Get dimensions of feet
        auto leftLink = urdfModel.getLink("leg_left_6_link");
        auto rightLink = urdfModel.getLink("leg_right_6_link");
        auto foot = boost::dynamic_pointer_cast<urdf::Box>(leftLink->collision->geometry);
        double length = foot->dim.z;
        double width = foot->dim.y;
        double x_offset = leftLink->collision->origin.position.z;
        double y_L_offset = leftLink->collision->origin.position.y;
        double y_R_offset = rightLink->collision->origin.position.y;

        // Get Transformation to feet
        KDL::Frame left_tf = getTF(kdlModel, "base_link", "left_sole_link");
        KDL::Frame right_tf = getTF(kdlModel, "base_link", "right_sole_link");

        KDL::Vector p1 = { x_offset + length/2, y_L_offset + width/2, 0 };
        p1 = left_tf * p1;
        KDL::Vector p2 = { x_offset - length/2, y_L_offset + width/2, 0 };
        p2 = left_tf * p2;
        KDL::Vector p3 = { x_offset - length/2, y_R_offset - width/2, 0 };
        p3 = right_tf * p3;
        KDL::Vector p4 = { x_offset + length/2, y_R_offset - width/2, 0 };
        p4 = right_tf * p4;

        /* vector<KDL::Vector> pts = {p1, p2, p3, p4};
        for(auto p : pts)
        {
            geometry_msgs::Point32 pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = p.z();
            supportBase.header.frame_id = "/base_link";
            supportBase.header.stamp = ros::Time::now();
            supportBase.polygon.points.push_back(pt);
        }
        sb_pub.publish(supportBase); */

        KDL::Frame world = left_tf;
        //world.M = left_tf.M;
        world.p.x((p1.x() + p2.x())/2);
        world.p.y((p1.y() + p4.y())/2);
        world.p.z(p1.z());

        world = world.Inverse();

        /* tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(world.p.x(), world.p.y(), world.p.z()) );
        double x, y, z, w;
        world.M.GetQuaternion(x, y, z, w);
        transform.setRotation( tf::Quaternion(x, y, z, w) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link")); */

        return world;
    }

    KDL::Vector ZMPController::computeCOM(KDL::Tree& model)
    {
        KDL::Frame tf = KDL::Frame::Identity();
        KDL::Vector com = KDL::Vector::Zero();
        double M = 0;
        recursiveCOM(model.getRootSegment(), tf, com, M);
        com = com/M;
        return com;
    }

    KDL::Vector ZMPController::computeZMP(KDL::Tree& model)
    {
        KDL::Vector zmp = KDL::Vector::Zero();

        double fz_total = sensors_["left_ft_sensor"].getForce()[2] + sensors_["right_ft_sensor"].getForce()[2];

        // Filter Coefficients for 10 Hz Low Pass filter
        static vector<double> a =
        {
                                    1.0,
                                    - 5.757244186246575523568935750518,
                                    13.815510806058025394804644747637,
                                    -17.68737617989402721718761313241,
                                    12.741617329229226740494596015196,
                                    - 4.896924891433742210722357413033,
                                    0.78441717688930268082003749441355
        };
        static vector<double> b =
        {
                                    0.00000000085315951525721800408064154908061,
                                    0.0000000051189570915433080244838492944837,
                                    0.000000012797392728858270061209623236209,
                                    0.000000017063190305144360081612830981612,
                                    0.000000012797392728858270061209623236209,
                                    0.0000000051189570915433080244838492944837,
                                    0.00000000085315951525721800408064154908061
        };
        static KDL::Vector zero = KDL::Vector::Zero();
        static butter<KDL::Vector> LPfilter(a,b,zero);
        for(auto sensor_name : sensor_names)
        {
            KDL::Vector trq = KDL::Vector::Zero();
            std::copy(sensors_[sensor_name.first].getTorque(),
                      sensors_[sensor_name.first].getTorque() + 3,
                      std::begin(trq.data));

            auto tf = getTF(model, "base_link", sensor_name.second);
            double fz = sensors_[sensor_name.first].getForce()[2];
            KDL::Vector tmp = KDL::Vector::Zero();
            tmp[0] = -trq[1]/fz;
            tmp[1] =  trq[0]/fz;

            tmp = tf * tmp;
            zmp += tmp * fz/fz_total;
        }
        zmp = LPfilter(zmp);
        return zmp;
    }

    KDL::Vector ZMPController::computeCP(KDL::Vector& com)
    {
        double g = 9.81;
        double z = 0.82;
        double w = sqrt(g/z);
        double dt = 0.001;

        KDL::Vector x = com;
        static numeric::derivative<KDL::Vector> dx(dt, x);
        KDL::Vector xd = dx(com);

        // compute capture point
        KDL::Vector cp = com + xd/w;
        return cp;
    }

    KDL::JntArray ZMPController::getChainState(KDL::Chain& chain)
    {
        int i = 0;
        KDL::JntArray q(chain.getNrOfJoints());
        for (auto segment: chain.segments)
        {
            if(segment.getJoint().getType() != KDL::Joint::JointType::None)
            {
                string joint_name = segment.getJoint().getName();
                double pos = joints_[joint_name].getPosition();
                q.data[i] = pos;
                i++;
            }
        }
        return q;
    }

    KDL::JntArray ZMPController::getChainCommand(KDL::Chain& chain)
    {
        int i = 0;
        KDL::JntArray q(chain.getNrOfJoints());
        for (auto segment: chain.segments)
        {
            if(segment.getJoint().getType() != KDL::Joint::JointType::None)
            {
                string joint_name = segment.getJoint().getName();
                double pos = joints_[joint_name].getCommand();
                q.data[i] = pos;
                i++;
            }
        }
        return q;
    }

    void ZMPController::commandComPos(KDL::Tree& model, double x)
    {
        // Left Leg kinematic chain
        model.getChain("left_sole_link", "base_link", kinematicChains["left"]);
        // Right Leg kinematic chain
        model.getChain("right_sole_link", "base_link", kinematicChains["right"]);
        KDL::Frame X_d;
        for(auto chain: kinematicChains)
        {
            KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain.second);
            KDL::ChainIkSolverPos_LMA iksolver = KDL::ChainIkSolverPos_LMA(chain.second);
            KDL::JntArray q = getChainCommand(chain.second);
            fksolver.JntToCart(q, X_d);
            X_d.p.x(x);
            KDL::JntArray q_des;
            iksolver.CartToJnt(q, X_d , q_des);
            setChainState(chain.second, q_des);
        }
    }

    void ZMPController::setChainState(KDL::Chain& chain, KDL::JntArray& q_d)
    {
        int i = 0;
        for(auto segment: chain.segments)
        {
            if(segment.getJoint().getType() != KDL::Joint::JointType::None)
            {
                string joint_name = segment.getJoint().getName();
                joints_[joint_name].setCommand(q_d.data[i]);
                i++;
            }
        }
    }

    double ZMPController::p_controller(KDL::Vector com, KDL::Vector zmp)
    {
        double k = 1;
        double p = zmp[0];
        double x = com[0];

        double vel = - k * (x - p) -  k * x;
        return vel;
    }

    double ZMPController::lqr_acc_controller(KDL::Vector com, KDL::Vector zmp, double xd)
    {
        const double k1 =  -88.2584;
        const double k2 =  190.7279;
        const double k3 =   59.3037;


        double p = zmp[0];
        double x = com[0];
        double v = xd;

        double a = - k1*p - k2*x - k3*v;

        return a;

    }

    double ZMPController::lqg_acc_controller(KDL::Vector com, KDL::Vector zmp)
    {
        double zc = 0.82;
        double g = 9.81;
        double dt = 0.001;
        static double u = 0;

        // Kalman State Estimator
        static Eigen::MatrixXd A(3,3), B(3,1), C(1,3), Q(3,3), R(1,1);

        A << 1, dt, pow(dt,2)/2,
                0,  1,     dt     ,
                0,  0,      1     ;

        B << pow(dt,2)/2, dt, 1;

        //C << 1 ,  0, -zc/g;
        C << 1 ,  0, -zc/g;

        Q << pow(dt,4)/4,   pow(dt,3)/2,   pow(dt,2)/2,
             pow(dt,3)/2,   pow(dt,2)  ,            dt,
             pow(dt,2)/2,            dt,             1;
        Q = Q*1;
        R << 0.001;

        static KalmanFilter kf(A,B,C,Q,R);

        Eigen::VectorXd y(1);
        y << zmp[0];
        auto x_hat = kf(y, u);

        // LQR Controller
        VectorXd K(3);
        K << 2.8930,    2.4776,    0.9163;
        u = (- K.transpose() * x_hat)(0);
        return u;
    }

    double ZMPController::lqr_jerk_controller(double x, double v, double a)
    {
        /* static double x = 0;//com[0];
        static double v = 0;
        static double a = 0;
        static numeric::derivative<double> dx(0.001,0);
        static numeric::derivative<double> dv(0.001,0);

        x = zmp[0];
        v = dx(x);
        a = dv(v);

        // Filter Coefficients for 10 Hz Low Pass filter
        static vector<double> f1 =
        {
                                    1.0,
                                    - 5.757244186246575523568935750518,
                                    13.815510806058025394804644747637,
                                    -17.68737617989402721718761313241,
                                    12.741617329229226740494596015196,
                                    - 4.896924891433742210722357413033,
                                    0.78441717688930268082003749441355
        };
        static vector<double> f2 =
        {
                                    0.00000000085315951525721800408064154908061,
                                    0.0000000051189570915433080244838492944837,
                                    0.000000012797392728858270061209623236209,
                                    0.000000017063190305144360081612830981612,
                                    0.000000012797392728858270061209623236209,
                                    0.0000000051189570915433080244838492944837,
                                    0.00000000085315951525721800408064154908061
        };
        static butter<double> LP(f1, f2, 0.0);
        a = LP(a); */



        // Controller
        double k1 = 0.4989 ;
        double k2 = 49.9920 ;
        double k3 = 10.0096 ;
        double u = - k1 * x - k2 * v - k3 * a;


        return u;
    }

    double ZMPController::cp_controller(KDL::Vector zmp, KDL::Vector zmp_ref)
    {
        //numeric::derivative<double> dt(0.001, 0);
        //numeric::derivative<double> dt_ref(0.001, 0);
        double p     = zmp.x();
        double p_ref = zmp_ref.x();
        //double pd = dt(p);
        //double pd_ref = dt_ref(p_ref);
        double kp = 1.5;
        //double kd = 0.00001;

        double vel = kp*(p-p_ref);// + kd*(pd-pd_ref);
        return vel;
    }




    KDL::Frame ZMPController::getTF(KDL::Tree& model, string startlink, string endlink)
    {
        KDL::Frame TF = KDL::Frame::Identity();
        KDL::Chain chain;
        model.getChain(startlink, endlink, chain);
        for(auto segment: chain.segments)
        {
            double q = 0;
            if(segment.getJoint().getType() != KDL::Joint::JointType::None)
            {
                string jointname = segment.getJoint().getName();
                q = joints_[jointname].getPosition();
            }
            TF = TF * segment.pose(q);
        }
        return TF;
    }

    void ZMPController::recursiveCOM(KDL::SegmentMap::const_iterator segment,
                                     KDL::Frame& tf,
                                     KDL::Vector& com,
                                     double& M)
    {
        double q = 0.0;
        // If Joint is not fixed, get positions
        if(segment->second.segment.getJoint().getType() != KDL::Joint::JointType::None)
        {
            string joint_name = segment->second.segment.getJoint().getName();
            q = joints_[joint_name].getPosition();
        }

        // Compute tf to current segment
        KDL::Frame curr_tf = tf * segment->second.segment.pose(q);
        // Add current segement to com
        KDL::Vector curr_cog = segment->second.segment.getInertia().getCOG();
        double m = segment->second.segment.getInertia().getMass();
        M += m;
        com += m * (curr_tf*curr_cog);

        for(auto child : segment->second.children)
        {
            recursiveCOM(child, curr_tf, com, M);
        }

    }

    void ZMPController::writeToCSV(std::string path, std::vector<VectorXd> &data)
    {
        std::ofstream file;
        file.open(path);
        for(auto vec: data)
        {
            file << vec.format(csv);
        }
        file.close();
        ROS_INFO("Written File");
    }

    int ZMPController::recordData(string filename, int start, int samples, VectorXd& dataSample)
    {
        string basepath = "/home/catkin_ws/src/reemc_balance_control/data/";
        string path = basepath + filename;
        static int sample = 0;
        static vector<VectorXd> out;
        if(sample == samples)
        {
            writeToCSV(path, out);
            exit(0);
        }
        else if(sample >= start)
        {
            out.push_back(dataSample);
        }
        sample++;
        return sample;
    }

    void ZMPController::visualize_KDLVector(KDL::Vector v)
    {
        viz_msg.header.stamp = ros::Time::now();
        viz_msg.type = visualization_msgs::Marker::SPHERE;
        viz_msg.action = visualization_msgs::Marker::ADD;
        viz_msg.pose.orientation.w = 1.0;

        viz_msg.scale.x = 0.025;
        viz_msg.scale.y = 0.025;
        viz_msg.scale.z = 0.025;
        viz_msg.color.a = 1.0;
        viz_msg.color.r = 1.0;
        viz_msg.color.g = 1.0;
        viz_msg.color.b = 1.0;

        viz_msg.header.frame_id = "/base_link";

        viz_msg.pose.position.x = v[0];
        viz_msg.pose.position.y = v[1];
        viz_msg.pose.position.z = v[2];

        v_pub.publish(viz_msg);

    }

    void ZMPController::visualize_zmp()
    {
        viz_msg.header.stamp = ros::Time::now();

        viz_msg.type = visualization_msgs::Marker::SPHERE;
        viz_msg.action = visualization_msgs::Marker::ADD;
        viz_msg.pose.orientation.w = 1.0;

        viz_msg.scale.x = 0.025;
        viz_msg.scale.y = 0.025;
        viz_msg.scale.z = 0.001;
        viz_msg.color.a = 1.0;
        viz_msg.color.r = 0.0;
        viz_msg.color.g = 1.0;
        viz_msg.color.b = 0.0;

        viz_msg.header.frame_id = "/left_sole_link";

        KDL::Frame left_tf = getTF(*KDLmodel, "base_link", "left_sole_link");
        KDL::Vector diff = world.Inverse().p - left_tf.p;

        viz_msg.pose.position.x = zmp[0] + diff[0];
        viz_msg.pose.position.y = zmp[1] + diff[1];
        viz_msg.pose.position.z = 0;
        zmp_pub.publish(viz_msg);
    }

    void ZMPController::visualize_com()
    {
        viz_msg.header.stamp = ros::Time::now();

        viz_msg.type = visualization_msgs::Marker::SPHERE;
        viz_msg.action = visualization_msgs::Marker::ADD;
        viz_msg.pose.orientation.w = 1.0;

        viz_msg.scale.x = 0.020;
        viz_msg.scale.y = 0.020;
        viz_msg.scale.z = 0.002;
        viz_msg.color.a = 1.0;
        viz_msg.color.r = 0.0;
        viz_msg.color.g = 0.0;
        viz_msg.color.b = 1.0;

        viz_msg.header.frame_id = "/left_sole_link";

        KDL::Frame left_tf = getTF(*KDLmodel, "base_link", "left_sole_link");
        KDL::Vector diff = world.Inverse().p - left_tf.p;

        viz_msg.pose.position.x = com[0] + diff[0];
        viz_msg.pose.position.y = com[1] + diff[1];
        viz_msg.pose.position.z = 0;

        com_pub.publish(viz_msg);
    }

    void ZMPController::visualize_cp()
    {
        viz_msg.header.stamp = ros::Time::now();

        viz_msg.type = visualization_msgs::Marker::SPHERE;
        viz_msg.action = visualization_msgs::Marker::ADD;
        viz_msg.pose.orientation.w = 1.0;

        viz_msg.scale.x = 0.019;
        viz_msg.scale.y = 0.019;
        viz_msg.scale.z = 0.003;
        viz_msg.color.a = 1.0;
        viz_msg.color.r = 1.0;
        viz_msg.color.g = 0.0;
        viz_msg.color.b = 0.0;

        viz_msg.header.frame_id = "/left_sole_link";

        KDL::Frame left_tf = getTF(*KDLmodel, "base_link", "left_sole_link");
        KDL::Vector diff = world.Inverse().p - left_tf.p;

        viz_msg.pose.position.x = cp[0]+diff[0];
        viz_msg.pose.position.y = cp[1]+diff[1];
        viz_msg.pose.position.z = 0;

        cp_pub.publish(viz_msg);
    }

    void ZMPController::apply_froce(double force)
    {
        gazebo_msgs::ApplyBodyWrench applyForce;
        applyForce.request.reference_frame = "base_link";
        applyForce.request.body_name = "torso_1_link";
        applyForce.request.duration.sec = 1;
        applyForce.request.wrench.force.x = force;
        applyForceClient.call(applyForce);
    }

}