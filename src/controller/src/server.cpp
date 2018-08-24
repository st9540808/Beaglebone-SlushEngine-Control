#include <algorithm>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>

class Action {
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    
    // publish fake joint states
    ros::Publisher controllerJointStatesPub_;
    uint32_t seq;

public:
    Action(std::string name) :
        as_(nh_, name, boost::bind(&Action::executeCB, this, _1), false),
        action_name_(name),
        controllerJointStatesPub_(nh_.advertise<sensor_msgs::JointState>(
                "/move_group/fake_controller_joint_states", 10
            )
        ),
        seq(0)
    {
        as_.start();
    }

    ~Action(void)
    {}

    void pubToJointStatePublisher(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
    {
        sensor_msgs::JointState js;

        js.header.seq = seq++;
        js.header.stamp = ros::Time::now();
        js.name = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};
        
        js.position.resize(goal->trajectory.points[0].positions.size());
        std::copy(goal->trajectory.points.back().positions.begin(),
                  goal->trajectory.points.back().positions.end(),
                  js.position.begin());
        js.velocity = {.0, .0, .0, .0, .0};
        
        controllerJointStatesPub_.publish(js);
    }

    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
    {
        // helper variables
        ros::Rate r(20);
        bool success = true;

        for (int i = 0; i < goal->trajectory.points.size(); i++) {
            for (auto pos : goal->trajectory.points[i].positions)
                std::printf("%f ", pos);
            std::puts("");
        }

        ROS_INFO("%s: Executing", action_name_.c_str());

        
        for (int i = 0; i < goal->trajectory.points.size(); i++) {
            feedback_.actual.positions.resize(5);
            feedback_.actual.positions[0] = goal->trajectory.points[i].positions[0];
            feedback_.actual.positions[1] = goal->trajectory.points[i].positions[1];
            feedback_.actual.positions[2] = goal->trajectory.points[i].positions[2];
            feedback_.actual.positions[3] = goal->trajectory.points[i].positions[3];
            feedback_.actual.positions[4] = goal->trajectory.points[i].positions[4];
            as_.publishFeedback(feedback_);
            r.sleep();
        }

        if (!success)
            goto failure_clearup;
            
        result_.error_code = 0;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
        pubToJointStatePublisher(goal);
        return;
    
    failure_clearup:
        ROS_INFO("failed in executeCB");
    }
};


int main(int argc, char* argv[])
{
    struct sched_param schedp;
    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }
    
    ROS_INFO("in main function");
    
    ros::init(argc, argv, "moveo_arm_controller");
    Action act("moveo_arm_controller/follow_joint_trajectory");
    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

