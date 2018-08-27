extern "C" { 
    #include <unistd.h>
    #include <sched.h>
}

#include <algorithm>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <motors/slushboard.h>
#include <motors/slushmotor.h>
#include <motors/gpio_pin.h>

#define SHOULDER_MICROSTEPS 2
#define remap_to_shoulder_steps(pos) ((pos) / (16 / SHOULDER_MICROSTEPS))
#define restore_to_shoulder_pos(remapped) ((remapped) * (16 / SHOULDER_MICROSTEPS))

class MultiStepper {
private:
    std::vector<int> posFrom, posTo;    // stepper positions (unit: step)
    std::vector<double> curFrom, curTo; // positions in radius
    
    // joint state (unit: rad)
    trajectory_msgs::JointTrajectoryPoint state;

    SlushBoard board;
    SlushMotor axis0, axis1_1, axis1_2, axis2, axis3, axis4;
    std::vector<SlushMotor*> allAxesPtr;
    std::vector<SlushMotor*> allJointsPtr; // exclude the second motor on shoulder

    // position in motor is not likely to be the same as posTo. So we need to remap and restore;
    std::vector<std::function<int(int)>> remapFunc = {
        [](int steps) { return  steps; },
        [](int steps) { return  remap_to_shoulder_steps(steps); },
        [](int steps) { return  steps; },
        [](int steps) { return  steps; },
        [](int steps) { return -steps; },
    };
    std::vector<std::function<int(int)>> restoreFunc = {
        [](int steps) { return  steps; },
        [](int steps) { return  restore_to_shoulder_pos(steps); },
        [](int steps) { return  steps; },
        [](int steps) { return  steps; },
        [](int steps) { return -steps; },
    };
    // using 1/16 micro steps
    const std::vector<int> stepsPerRevolution {32800, 18000, 72000, 3200, 14400, 0};
    const std::vector<int>   microSteps {16, SHOULDER_MICROSTEPS, 16, 16, 16};
    const std::vector<float> defaultSpd {150, 35, 300, 45, 200};

public:
    MultiStepper(void)
        : posFrom(5, 0), posTo(5, 0)
        , axis0(0), axis1_1(1), axis1_2(2), axis2(3), axis3(4), axis4(5)
        , allAxesPtr({&axis0, &axis1_1, &axis1_2, &axis2, &axis3, &axis4})
        , allJointsPtr({&axis0, &axis1_1, &axis2, &axis3, &axis4})
    {
        for (auto axisPtr : allAxesPtr)
            axisPtr->resetDev();
        setMicroSteps(this->microSteps);
        setMaxSpeed(this->defaultSpd);
 
        setKVAL(&axis1_1, {55, 65, 73, 73});
        setKVAL(&axis1_2, {55, 65, 73, 73});
        setKVAL(&axis4,   {30, 35, 41, 41});
        axis1_1.setAcc(100);
        axis1_1.setDec(100);
        axis1_2.setAcc(100);
        axis1_2.setDec(100);

        for (auto axisPtr : allAxesPtr)
            axisPtr->hardStop();
    }

    void setMicroSteps(std::vector<int> stp)
    {
        if (stp.size() != 5) {
            ROS_ERROR("wrong size in microSteps: %d", stp.size());
            return;
        }
        stp.insert(stp.begin()+1, stp[1]);
        for (int i = 0; i < 6; i++)
            allAxesPtr[i]->setMicroSteps(stp[i]);
    }

    void setMaxSpeed(std::vector<float> spd)
    {
        if (spd.size() != 5) {
            ROS_ERROR("wrong size in speed: %d", spd.size());
            return;
        }
        spd.insert(spd.begin()+1, spd[1]);
        for (int i = 0; i < 6; i++)
            allAxesPtr[i]->setMaxSpeed(spd[i]);
    }

    void setKVAL(SlushMotor* axis, const std::vector<int>& kval)
    {
        if (kval.size() != 4) {
            ROS_ERROR("wrong size in kval: %d", kval.size());
            return;
        }
        axis->setHoldKVAL(kval[0]);
        axis->setRunKVAL(kval[1]);
        axis->setAccKVAL(kval[2]);
        axis->setDecKVAL(kval[3]);
    }

    void move(const std::vector<double>& from, const std::vector<double>& to)
    {
        std::vector<int> armSteps(5), remapped(5);
        
        curFrom = from;
        curTo = to;

        // assign previous goal state to start state
        for (int i = 0; i < 5; i++) {
            // restore from the steps that motor actually moved
            posFrom[i] = restoreFunc[i](allJointsPtr[i]->getPos());
        }

        for (int i = 0; i < 5; i++) {
            // calculate steps to move
            armSteps[i] = static_cast<int>(
                (to[i] - from[i]) * stepsPerRevolution[i] / (2. * M_PI)
            );
        }
        for (int i = 0; i < 5; i++) {
            // update position for each joint
            posTo[i] = posFrom[i] + armSteps[i];
        }

        ROS_INFO("posFrom: %d %d %d %d %d", posFrom[0], posFrom[1], posFrom[2], posFrom[3], posFrom[4]);
        ROS_INFO("posTo: %d %d %d %d %d", posTo[0], posTo[1], posTo[2], posTo[3], posTo[4]);

        for (int i = 0; i < 5; i++) {
            // remap positions
            remapped[i] = remapFunc[i](posTo[i]);
        }

        setMaxSpeed(defaultSpd);
        setMaxSpeed(calculateSpd(armSteps));
        axis0.goTo(remapped[0]);
        axis1_1.goTo(remapped[1]);
        axis1_2.goTo(-remapped[1]);
        axis2.goTo(remapped[2]);
        axis3.goTo(remapped[3]);
        axis4.goTo(remapped[4]);
    }

    std::vector<float> calculateSpd(std::vector<int> armSteps)
    {
        float acc[4], dec[4], spd[4], longestTime = .0f;
        SlushMotor* axes[4] = { &axis0, &axis1_1, &axis2, &axis4 };
        std::vector<float> newSpd(4);

        // exclude axis3
        armSteps.erase(armSteps.begin()+3);
        for (int i = 0; i < 4; i++)
            armSteps[i] = std::abs(armSteps[i] / 16);

        for (int i = 0; i < 4; i++) {
            acc[i] = axes[i]->getAcc();
            dec[i] = axes[i]->getDec();
            spd[i] = axes[i]->getMaxSpeed();
            // ROS_INFO("axis%d: %f %f %f", i, acc[i], dec[i], spd[i]);
        }
        for (int i = 0; i < 4; i++) {
            float thisTime = (spd[i]/acc[i] + spd[i]/dec[i])/2 +
                             (float) armSteps[i]/spd[i];
            longestTime = std::max(thisTime, longestTime);
            // ROS_INFO("axis%d thisTime: %f", i, thisTime);
        }
        ROS_INFO("longestTime: %f", longestTime);

        if (longestTime > .0f) {
            for (int i = 0; i < 4; i++) {
                newSpd[i] = (float) armSteps[i] / longestTime;
                newSpd[i] = std::min(newSpd[i], 300.0f); // safety check
            }
            
            newSpd.insert(newSpd.begin()+3, defaultSpd[3]);
            ROS_INFO("newSpd: %f %f %f %f %f",
                      newSpd[0], newSpd[1], newSpd[2], newSpd[3], newSpd[4]);
            return newSpd;
        }
        return defaultSpd;
    }

    void join(void)
    {
        for (auto axisPtr : allAxesPtr)
            axisPtr->join();
    }

    void emergencyStop(void)
    {
        for (auto axisPtr : allAxesPtr)
            axisPtr->softStop();
        join();
    }

    // check all joints has stopped
    bool isStopped(void)
    {
        return std::none_of(
                   allAxesPtr.begin(), allAxesPtr.end(),
                   [](SlushMotor* axisPtr) { return axisPtr->isBusy(); }
               );
    }

    trajectory_msgs::JointTrajectoryPoint getCurrentState(void)
    {
        auto step2rad = [this](SlushMotor* axis, int jointNum) -> double {
            long absPos = restoreFunc[jointNum](axis->getPos());
            double diff = curTo[jointNum] - curFrom[jointNum];
            return curFrom[jointNum] + (
                       (double) std::abs(absPos - posFrom[jointNum]) /
                       std::max(std::abs(posTo[jointNum] - posFrom[jointNum]), 1)
                       // using max to prevent divide by zero
                   ) * diff;
        };
        auto _2angular = [this](SlushMotor* axis, int jointNum) -> double {
            double stepsPerSec = 16 * axis->getCurrentSpeed();
            return stepsPerSec / stepsPerRevolution[jointNum] * (2.*M_PI);
        };
        static const std::array<int, 5> jointNums {0, 1, 2, 3, 4};
        
        state.positions.resize(5);
        std::transform(
            allJointsPtr.begin(), allJointsPtr.end(), jointNums.begin(),
            state.positions.begin(),
            [&](SlushMotor* axis, int jointNum) { return step2rad(axis, jointNum); }
        );

        state.velocities.resize(5);
        std::transform(
            allJointsPtr.begin(), allJointsPtr.end(), jointNums.begin(),
            state.velocities.begin(),
            [&](SlushMotor* axis, int jointNum) { return _2angular(axis, jointNum); }
        );             

        std::printf("stepper pos: ");
        for (auto axis : allJointsPtr)
            printf("%-6d ", axis->getPos());
        std::printf("\n");
        std::printf("stepper vel: ");
        for (auto axis : allJointsPtr)
            printf("%-6.2f ", axis->getCurrentSpeed());
        std::printf("\n");
        std::printf("joint pos: ");
        for (auto pos : state.positions) {
            if (pos >= 0) std::printf(" %-8.4f ", pos);
            else std::printf("%-9.4f ", pos);
        }
        std::printf("\n");
        std::printf("joint vel: ");
        for (auto spd : state.velocities) {
            if (spd >= 0) std::printf(" %-8.4f ", spd);
            else std::printf("%-9.4f ", spd);
        }
        std::printf("\n");
        std::printf("----------------------------------------------------\n");

        return this->state;
    }
};


class ArmController {
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
    uint32_t seq_;

    // stepper control
    MultiStepper arm;

public:
    ArmController(std::string name) :
        as_(nh_, name, boost::bind(&ArmController::executeCB, this, _1), false),
        action_name_(name),
        controllerJointStatesPub_(nh_.advertise<sensor_msgs::JointState>(
                "/move_group/fake_controller_joint_states", 10
            )
        ),
        seq_(0)
    {
        as_.start();
    }

    ~ArmController(void)
    {}

    void pubToJointStatePublisher(const trajectory_msgs::JointTrajectoryPoint& point)
    {
        sensor_msgs::JointState js;

        js.header.seq = seq_++;
        js.header.stamp = ros::Time::now();
        js.name = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};
        
        js.position.resize(point.positions.size());
        js.velocity.resize(point.velocities.size());
        std::copy(point.positions.begin(), point.positions.end(), js.position.begin());
        std::copy(point.velocities.begin(), point.velocities.end(), js.velocity.begin());
        
        controllerJointStatesPub_.publish(js);
    }

    void cancelExecution(void)
    {
        arm.emergencyStop();
        ROS_WARN("emergency stop for all joints");
    }

    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
    {
        // helper variables
        int i = 0;
        bool success = true;
        ros::Rate r(4);
        
        const std::vector<double>& from = goal->trajectory.points.front().positions;
        const std::vector<double>& to = goal->trajectory.points.back().positions;

        ROS_INFO("%s: Executing", action_name_.c_str());
        ROS_INFO("from: %f %f %f %f %f", from[0], from[1], from[2], from[3], from[4]);
        ROS_INFO("to: %f %f %f %f %f", to[0], to[1], to[2], to[3], to[4]);

        arm.move(from, to);
        while (!arm.isStopped()) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                cancelExecution();
                success = false;
                break;
            }
            arm.getCurrentState();
            r.sleep();
        }

        if (success) {
            result_.error_code = 0;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
            pubToJointStatePublisher(arm.getCurrentState());
        } else {
            result_.error_code = 0;
            ROS_WARN("failed in executeCB");
            as_.setPreempted(result_);
            pubToJointStatePublisher(arm.getCurrentState());
        }
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
    ArmController armController("moveo_arm_controller/follow_joint_trajectory");
    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

