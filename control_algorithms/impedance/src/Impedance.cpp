#include "impedance/Impedance.h"
#include "impedance/KDL_Base.h"
#include "kdl_conversions/kdl_msg.h"
#include <math.h>
#include <fstream>


void Impedance::init(ros::NodeHandle &nh,
        std::string topic_arm_state,
        std::string topic_arm_command,
        std::string topic_wrench_state,
        std::vector<double> Ka,
        std::vector<double> Kv,
        std::vector<double> Kp,
        std::vector<double> M,
        std::vector<double> D,
        std::vector<double> K,
        std::vector<double> desired_pose)
{
    //* Subscribers
    sub_arm_state_           = nh_.subscribe(topic_arm_state, 5,
        &Impedance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
        &Impedance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_posture_ = nh_.subscribe("/joint_torque_controller/command", 1,
        &Impedance::command,this, ros::TransportHints().reliable().tcpNoDelay());
    //* Publishers
    pub_arm_cmd_              = nh_.advertise<joint_effort_msg::JointEfforts>(topic_arm_command, 5);

    // KDL
    kdl_base::KDL_Base::init(nh);
    Gravity = KDL::Vector::Zero();
    Gravity(2) = -9.81;

    Kp_.resize(this->kdl_chain_.getNrOfJoints());
	Kv_.resize(this->kdl_chain_.getNrOfJoints());
    Ka_.resize(this->kdl_chain_.getNrOfJoints());
	M_.resize(this->kdl_chain_.getNrOfJoints());
	C_.resize(this->kdl_chain_.getNrOfJoints());
	G_.resize(this->kdl_chain_.getNrOfJoints());

    // KDL: Kinematics
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
    // KDL::ChainIkSolverVel_pinv_givens ik_vel_solver_ = KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_);
    // KDL::ChainFkSolverPos_recursive fk_pos_solver_ = KDL::ChainFkSolverPos_recursive(this->kdl_chain_);
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR(this->kdl_chain_, *(fk_pos_solver_.get()), *(ik_vel_solver_.get())));
    desired_pose_ = desired_pose;
    Desired_Pos_ = KDL::Vector(desired_pose[0], desired_pose[1], desired_pose[2]);
    Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose[3], desired_pose[4], desired_pose[5],desired_pose[6]);
    Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
    Desired_Vel_ = KDL::Twist::Zero(); 
    jac_solver_.reset(new KDL::ChainJntToJacSolver(this->kdl_chain_));

    // KDL: Dynamics
    id_pos_solver_.reset(new KDL::ChainIdSolver_RNE(this->kdl_chain_, Gravity));
    id_solver_.reset( new KDL::ChainDynParam(this->kdl_chain_, Gravity));

    // get publishing period
    if (!nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
    }

    // Variable init
    Jnt_Pos_Init_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Desired_State.resize(this->kdl_chain_.getNrOfJoints());
    CMD_State.resize(this->kdl_chain_.getNrOfJoints());
    Current_State.resize(this->kdl_chain_.getNrOfJoints());

    Jnt_Pos_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Vel_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Toq_State.resize(this->kdl_chain_.getNrOfJoints());

    Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints());

    Ext_Wrenches.resize(kdl_chain_.getNrOfSegments());
    KDL::Wrench wrench = KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    Ext_Wrenches.back() = wrench;

    for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
    {
        Kp_(i) = Kp[i];
        Kv_(i) = Kv[i];
        Ka_(i) = Ka[i];
    }

    Impedance_M = M;
    Impedance_D = D;
    Impedance_K = K;

    Recieved_Joint_State = false;
    Cmd_Flag_            = true;
    Init_Flag_           = true;
    Step_                = 0;

    wrench_z             = 0;
    pos_z                = 0;
}
void Impedance::state_arm_callback(const joint_state_msg::JointState msg)
{
    for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
    {
        Jnt_Pos_State(i) = msg.position[i];
        Jnt_Vel_State(i) = msg.velocity[i];
        Jnt_Toq_State(i) = msg.effort[i];
        Recieved_Joint_State = true;
    }
    // std::cout<<Jnt_Pos_State(0)<<","<<Jnt_Pos_State(1)<<","<<Jnt_Pos_State(2)<<","
    // <<Jnt_Pos_State(3)<<","<<Jnt_Pos_State(4)<<","<<Jnt_Pos_State(5)<<std::endl;
    // std::cout<<"Recieved Joint State"<<std::endl;

}

void Impedance::state_wrench_callback(
  const geometry_msgs::WrenchConstPtr msg) {
    KDL::Wrench wrench = KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    Ext_Wrenches.back() = wrench;

    wrench_x = msg->force.x;
    wrench_y = msg->force.y;
    wrench_z = msg->force.z;
}

void Impedance::appendEigenVectorToFile(const Eigen::VectorXd& vec, const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::app);  // Open file in append mode
    if (file.is_open()) {
        for (int i = 0; i < vec.size(); ++i) {
            file << vec[i] << " ";
        }
        file << "\n";  // Newline to separate each vector
        file.close();
    } else {
        std::cerr << "Unable to open file";
    }
}

void Impedance::compute_impedance(bool flag)
{
    if(flag)
    {
        //! Method: 1
        if (Init_Flag_)
        {
            Jnt_Pos_Init_State = Jnt_Pos_State;
            Init_Flag_ = false;
        }

        // if (wrench_z != 0)
        // {
        //     KDL::Frame End_Pose;
        //     fk_pos_solver_->JntToCart(Jnt_Pos_State,End_Pose);

        //     KDL::JntArrayVel Jnt_Vel;
        //     KDL::FrameVel End_Pose_Vel;
        //     Jnt_Vel = KDL::JntArrayVel(Jnt_Pos_State, Jnt_Vel_State);
        //     fk_vel_solver_->JntToCart(Jnt_Vel, End_Pose_Vel);

        //     KDL::Vector pose_p, pose_vel_p;
        //     pose_p = End_Pose.p;
        //     pose_vel_p = End_Pose_Vel.p.p;

        //     // double acc_x = (wrench_x - (Impedance_D[0]*pose_vel_p(0) + Impedance_K[0]*(pose_p(0)-desired_pose_[0])))/Impedance_M[0];
        //     // double acc_y = (wrench_y - (Impedance_D[1]*pose_vel_p(1) + Impedance_K[1]*(pose_p(1)-desired_pose_[1])))/Impedance_M[1];
        //     double acc_z = (wrench_z - (Impedance_D[2]*pose_vel_p(2) + Impedance_K[2]*(desired_pose_[2]-pose_p(2))))/Impedance_M[2];

        //     ros::Rate loop_rate_(200);
        //     ros::Duration duration = loop_rate_.expectedCycleTime();
        //     // pos_x = pos_x + 0.01*(pose_vel_p(0) * duration.toSec() + 0.5 * acc_x * duration.toSec() * duration.toSec());
        //     // pos_y = pos_y + 0.01*(pose_vel_p(1) * duration.toSec() + 0.5 * acc_y * duration.toSec() * duration.toSec());
        //     pos_z = 10*(pose_vel_p(2) * duration.toSec() + 0.5 * acc_z * duration.toSec() * duration.toSec());
        //     Desired_Pos_ = KDL::Vector(desired_pose_[0]+pos_x, desired_pose_[1]+pos_y, desired_pose_[2]+pos_z);
        //     Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose_[3], desired_pose_[4], desired_pose_[5],desired_pose_[6]);
        //     Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
        // }

        ik_pos_solver_->CartToJnt(Jnt_Pos_State, Desired_Pose_, CMD_State);

        // Vector6d error;
        KDL::Frame Cur_Pose, Err_Pose;
        fk_pos_solver_->JntToCart(Jnt_Pos_State,Cur_Pose);
        Eigen::Vector3d Err_Pose_p = Eigen::Vector3d(Desired_Pose_.p.x() - Cur_Pose.p.x(),
                                             Desired_Pose_.p.y() - Cur_Pose.p.y(),
                                             Desired_Pose_.p.z() - Cur_Pose.p.z());
        std::cout << "Desired Pose:\t" << Desired_Pose_.p.x() << "\t" << Desired_Pose_.p.y() << "\t" << Desired_Pose_.p.z() << "\n";
        std::cout << "Current Pose:\t" << Cur_Pose.p.x() << "\t" << Cur_Pose.p.y() << "\t" << Cur_Pose.p.z() << "\n";
        // std::cout << "ok1\n";
        Eigen::Matrix3d Desired_Rot;
        Eigen::Matrix3d Cur_Rot;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Desired_Rot(i, j) = Desired_Pose_.M(i, j);
                Cur_Rot(i, j) = Cur_Pose.M(i, j);
            }
        }
        Eigen::Matrix3d Err_Rot_Matrix = Desired_Rot * Cur_Rot.transpose();
        // Eigen::Matrix3d Err_Rot_Matrix = Cur_Rot * Desired_Rot.transpose();
        Eigen::AngleAxisd Err_Rot_AA(Err_Rot_Matrix);
        Eigen::Vector3d Err_Pose_Rot = Err_Rot_AA.angle() * Err_Rot_AA.axis();

        Eigen::VectorXd Err_Pose_eigen = Eigen::VectorXd::Zero(6);
        Err_Pose_eigen << Err_Pose_p, Err_Pose_Rot;
        // Err_Pose_eigen << Err_Pose_p;
        // std::cout << "ok2\n";
        std::cout << "Error Pose: ";
        for(int i = 0; i < 6; i++)
            std::cout << Err_Pose_eigen(i) << " ";
        std::cout << "\n";
        std::string pose_err_filename = "/home/akash/catkin_ws/controller_logs/pose_error.txt";
        appendEigenVectorToFile(Err_Pose_eigen, pose_err_filename);

        // Err_Pose.p = Desired_Pose_.p - Cur_Pose.p;
        // Err_Pose.M = Desired_Pose_.M * Cur_Pose.M.Inverse();

        KDL::JntArrayVel Jnt_Vel;
        KDL::FrameVel Cur_Vel;
        Jnt_Vel = KDL::JntArrayVel(Jnt_Pos_State, Jnt_Vel_State);
        fk_vel_solver_->JntToCart(Jnt_Vel, Cur_Vel);
        KDL::Twist Err_Vel = Desired_Vel_ - Cur_Vel.GetTwist();
        Eigen::VectorXd Err_Vel_eigen(6);
        Err_Vel_eigen.head<3>() = Eigen::Vector3d(Err_Vel.vel.x(), Err_Vel.vel.y(), Err_Vel.vel.z());
        Err_Vel_eigen.tail<3>() = Eigen::Vector3d(Err_Vel.rot.x(), Err_Vel.rot.y(), Err_Vel.rot.z());

        std::cout << "Desired Vel:\t" << Desired_Vel_.vel.x() << "\t" << Desired_Vel_.vel.y() << "\t" << Desired_Vel_.vel.z() << "\n";
        std::cout << "Current Vel:\t" << Cur_Vel.GetTwist().vel.x() << "\t" << Cur_Vel.GetTwist().vel.y() << "\t" << Cur_Vel.GetTwist().vel.z() << "\n";
        std::cout << "Error Vel: ";
        for(int i = 0; i < 6; i++)
            std::cout << Err_Vel_eigen(i) << " ";
        std::cout << "\n";

        id_solver_->JntToMass(Jnt_Pos_State, M_);
        id_solver_->JntToCoriolis(Jnt_Pos_State, Jnt_Vel_State, C_);
        id_solver_->JntToGravity(Jnt_Pos_State, G_);
        // std::cout << "ok3\n";

        KDL::Jacobian J(this->kdl_chain_.getNrOfJoints());
        int res = jac_solver_->JntToJac(Jnt_Pos_State,J);
        if (res < 0) {
            std::cerr << "Error: JntToJac failed with code " << res << std::endl;
            // Handle the error appropriately
        }
        // for(size_t i=0; i<this->kdl_chain_.getNrOfJoints(); i++)
        //     std::cout << Jnt_Pos_State(i) << "\t";
        
        // for (unsigned int i = 0; i < J.rows(); ++i) {
        //     for (unsigned int j = 0; j < J.columns(); ++j) {
        //         std::cout << J(i,j) << "\t";
        //     }
        //     std::cout << "\n";
        // }
        Eigen::MatrixXd J_eigen = J.data;
        // std::cout << "ok30" << J_eigen << "\n";
        Eigen::MatrixXd J_inv = J_eigen.completeOrthogonalDecomposition().pseudoInverse();
        // std::cout << "ok300\n";
        Eigen::MatrixXd J_inv_T = J_inv.transpose();
        // std::cout << "ok31\n";

        Eigen::MatrixXd M_cart = J_inv_T * M_.data * J_inv;
        // std::cout << "ok32\n";

        Eigen::VectorXd C_eigen = Eigen::VectorXd::Map(C_.data.data(), C_.rows());
        Eigen::VectorXd C_cart = J_inv_T * C_eigen;
        // std::cout << "ok33\n";

        Eigen::VectorXd G_eigen = Eigen::VectorXd::Map(G_.data.data(), G_.rows());
        Eigen::VectorXd G_cart = J_inv_T * G_eigen;
        // std::cout << "ok4\n";

        int sz = this->kdl_chain_.getNrOfJoints();
        Eigen::MatrixXd M_imp = Eigen::MatrixXd::Zero(sz,sz);
        Eigen::MatrixXd D_imp = Eigen::MatrixXd::Zero(sz,sz);
        Eigen::MatrixXd K_imp = Eigen::MatrixXd::Zero(sz,sz);
        for (int i = 0; i < sz; ++i) {
            M_imp(i, i) = Impedance_M[i];
            D_imp(i, i) = Impedance_D[i];
            K_imp(i, i) = Impedance_K[i];
        }
        // std::cout << "M imp\n";
        // for(int i = 0; i < sz; i++)
        // {
        //     for(int j = 0; j < sz; j++)
        //     {
        //         std::cout << M_imp(i,j) << "\t";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "D imp\n";
        // for(int i = 0; i < sz; i++)
        // {
        //     for(int j = 0; j < sz; j++)
        //     {
        //         std::cout << D_imp(i,j) << "\t";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "K imp\n";
        // for(int i = 0; i < sz; i++)
        // {
        //     for(int j = 0; j < sz; j++)
        //     {
        //         std::cout << K_imp(i,j) << "\t";
        //     }
        //     std::cout << "\n";
        // }

        Eigen::VectorXd Ext_Wrench_eigen = Eigen::VectorXd::Zero(6);
        Ext_Wrench_eigen << wrench_x, wrench_y, wrench_z;

        // if (!Ext_Wrenches.empty()) {
        //     KDL::Wrench last_wrench = Ext_Wrenches.back();
        //     Ext_Wrench_eigen << last_wrench.force.x(), last_wrench.force.y(), last_wrench.force.z(),
        //                         last_wrench.torque.x(), last_wrench.torque.y(), last_wrench.torque.z();
        // }
        
        std::cout << "Ext wrench: ";
        for(int i = 0; i < 6; i++)
            std::cout << Ext_Wrench_eigen(i) << " ";
        std::cout << "\n";

        Eigen::VectorXd acc = M_imp.inverse() * ( D_imp * Err_Vel_eigen + K_imp * Err_Pose_eigen + Ext_Wrench_eigen);

        std::cout << "Acc: ";
        for(int i = 0; i < 6; i++)
            std::cout << acc(i) << " ";
        std::cout << "\n";

        Eigen::VectorXd applied_tor = J_eigen.transpose() * (M_cart * acc + C_cart + G_cart - Ext_Wrench_eigen); 
        for (int i = 0; i < this->kdl_chain_.getNrOfJoints(); ++i)
            Jnt_Toq_Cmd_(i) = applied_tor(i);

        // // reaching desired joint position using a hyperbolic tangent function
        // double lambda = 0.1;
        // double th = tanh(M_PI - lambda*Step_);
        // double ch = cosh(M_PI - lambda*Step_);
        // double sh2 = 1.0/(ch*ch);

        // for(size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
        // {
        //     //take into account also initial/final velocity and acceleration
        //     Current_State(i) = CMD_State(i) - Jnt_Pos_Init_State(i);
        //     Jnt_Desired_State.q(i) = Current_State(i)*0.5*(1.0-th) + Jnt_Pos_Init_State(i);
        //     Jnt_Desired_State.qdot(i) = Current_State(i)*0.5*lambda*sh2;
        //     Jnt_Desired_State.qdotdot(i) = Current_State(i)*lambda*lambda*sh2*th;
        // }
        // // std::cout<<Current_State(0)<<","<<Current_State(1)<<","<<Current_State(2)<<","
        // // <<Current_State(3)<<","<<Current_State(4)<<","<<Current_State(5)<<std::endl;
        // ++Step_;
        // if(Jnt_Desired_State.q == CMD_State)
        // {
        //     Cmd_Flag_ = false;	//reset command flag
        //     Step_ = 0;
        //     Jnt_Pos_Init_State = Jnt_Pos_State;
        //     // ROS_INFO("Posture OK");
        // }

    	// // computing Inertia, Coriolis and Gravity matrices
        // id_solver_->JntToMass(Jnt_Pos_State, M_);
        // id_solver_->JntToCoriolis(Jnt_Pos_State, Jnt_Vel_State, C_);
        // id_solver_->JntToGravity(Jnt_Pos_State, G_);

        // // PID controller
        // KDL::JntArray pid_cmd_(this->kdl_chain_.getNrOfJoints());
        // // compensation of Coriolis and Gravity
        // KDL::JntArray cg_cmd_(this->kdl_chain_.getNrOfJoints());

        // for(size_t i=0; i<this->kdl_chain_.getNrOfJoints(); i++)
        // {
        //     // control law
        //     pid_cmd_(i) = Ka_(i)*Jnt_Desired_State.qdotdot(i) + Kv_(i)*(Jnt_Desired_State.qdot(i) - Jnt_Vel_State(i)) + Kp_(i)*(Jnt_Desired_State.q(i) - Jnt_Pos_State(i));
        //     cg_cmd_(i) = C_(i) + G_(i);
        //     // cg_cmd_(i) = C_(i)*Jnt_Desired_State.qdot(i) + G_(i);

        //     // Jnt_Toq_Cmd_(i) = M_(i)*Jnt_Desired_State.qdotdot(i)+C_(i)*Jnt_Desired_State.qdot(i)+G_(i);
        // }
        // Jnt_Toq_Cmd_.data = M_.data * pid_cmd_.data;
        // KDL::Add(Jnt_Toq_Cmd_,cg_cmd_,Jnt_Toq_Cmd_);


        // //! Method: 2 for Test
        // // if(id_pos_solver_->CartToJnt(Jnt_Pos_State, Jnt_Vel_State, Jnt_Acc_Cmd_, Ext_Wrenches, Jnt_Toq_Cmd_)!=0)
        // // {
        // //     ROS_ERROR("Could not compute joint torques! Setting all torques to zero!");
        // //     KDL::SetToZero(Jnt_Toq_Cmd_);
        // // }

        send_commands_to_robot();

        Recieved_Joint_State = false;
    }
}

void Impedance::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if(msg->data.size() == 0)
        ROS_INFO("Desired configuration must be of dimension %lu", this->kdl_chain_.getNrOfJoints());
    else if(msg->data.size() != this->kdl_chain_.getNrOfJoints())
    {
        ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
            CMD_State(i) = msg->data[i];
        std::cout<<"COmmand SET"<<std::endl;
        Cmd_Flag_ = true;
        // when a new command is set, steps should be reset to avoid jumps in the update
        Step_ = 0;
    }

}
void Impedance::send_commands_to_robot() {

    joint_effort_msg::JointEfforts   msg;

    msg.Joint1Effort = Jnt_Toq_Cmd_(0);
    msg.Joint2Effort = Jnt_Toq_Cmd_(1);
    msg.Joint3Effort = Jnt_Toq_Cmd_(2);
    msg.Joint4Effort = Jnt_Toq_Cmd_(3);
    msg.Joint5Effort = Jnt_Toq_Cmd_(4);
    msg.Joint6Effort = Jnt_Toq_Cmd_(5);
    // std::cout<<Jnt_Toq_Cmd_(0)<<","<<Jnt_Toq_Cmd_(1)<<","<<Jnt_Toq_Cmd_(2)<<","
    // <<Jnt_Toq_Cmd_(3)<<","<<Jnt_Toq_Cmd_(4)<<","<<Jnt_Toq_Cmd_(5)<<std::endl;
    pub_arm_cmd_.publish(msg);
}

void Impedance::run()
{
    ROS_INFO("Running the impedance control loop .................");
    ros::Rate loop_rate_(200);

    std::ofstream file;

    // Open the file in truncation mode to create a new file or clear the existing file
    std::string pose_err_filename = "/home/akash/catkin_ws/controller_logs/pose_error.txt";
    file.open(pose_err_filename, std::ios::trunc);
    if (!file) {
        std::cerr << "Error opening file in truncation mode." << std::endl;
        return;
    }
    // Close the file to reset it
    file.close();

    while (nh_.ok()) {

        compute_impedance(Recieved_Joint_State);

        ros::spinOnce();

        loop_rate_.sleep();
    }
}