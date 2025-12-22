#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class WheelbirdModeSwitcher 
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient arming_client1;
    ros::ServiceClient set_mode_client1;
    mavros_msgs::State current_state1;
    ros::Subscriber state_sub1;

    ros::ServiceClient arming_client2;
    ros::ServiceClient set_mode_client2;
    mavros_msgs::State current_state2;
    ros::Subscriber state_sub2;

    ros::ServiceClient arming_client3;
    ros::ServiceClient set_mode_client3;
    mavros_msgs::State current_state3;
    ros::Subscriber state_sub3;

    ros::ServiceClient arming_client4;
    ros::ServiceClient set_mode_client4;
    mavros_msgs::State current_state4;
    ros::Subscriber state_sub4;

    ros::ServiceClient arming_client5;
    ros::ServiceClient set_mode_client5;
    mavros_msgs::State current_state5;
    ros::Subscriber state_sub5;

    ros::ServiceClient arming_client6;
    ros::ServiceClient set_mode_client6;
    mavros_msgs::State current_state6;
    ros::Subscriber state_sub6;

public:
    WheelbirdModeSwitcher() 
    {
        arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird1/mavros/cmd/arming");
        set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird1/mavros/set_mode");
        state_sub1= nh.subscribe("wheelbird1/mavros/state", 10, &WheelbirdModeSwitcher::state_cb1, this);

        arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird2/mavros/cmd/arming");
        set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird2/mavros/set_mode");
        state_sub2 = nh.subscribe("wheelbird2/mavros/state", 10, &WheelbirdModeSwitcher::state_cb2, this);

        arming_client3 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird3/mavros/cmd/arming");
        set_mode_client3 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird3/mavros/set_mode");
        state_sub3 = nh.subscribe("wheelbird3/mavros/state", 10, &WheelbirdModeSwitcher::state_cb3, this);

        arming_client4 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird4/mavros/cmd/arming");
        set_mode_client4 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird4/mavros/set_mode");
        state_sub4= nh.subscribe("wheelbird4/mavros/state", 10, &WheelbirdModeSwitcher::state_cb4, this);

        arming_client5 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird5/mavros/cmd/arming");
        set_mode_client5 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird5/mavros/set_mode");
        state_sub5= nh.subscribe("wheelbird5/mavros/state", 10, &WheelbirdModeSwitcher::state_cb5, this);
    
        arming_client6 = nh.serviceClient<mavros_msgs::CommandBool>("wheelbird6/mavros/cmd/arming");
        set_mode_client6 = nh.serviceClient<mavros_msgs::SetMode>("wheelbird6/mavros/set_mode");
        state_sub6= nh.subscribe("wheelbird6/mavros/state", 10, &WheelbirdModeSwitcher::state_cb5, this);
    }

    void state_cb1(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state1 = *msg;
    }

    void state_cb2(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state2 = *msg;
    }

    void state_cb3(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state3 = *msg;
    }

    void state_cb4(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state4 = *msg;
    }

    void state_cb5(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state5 = *msg;
    }

    void state_cb6(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state6 = *msg;
    }

    int get_key_input() 
    {
        int ch;
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void spin() 
    {
        ROS_INFO("Press 1 to ARM, 2 to set OFFBOARD mode");
        while (ros::ok()) 
        {
            ros::spinOnce();
            int key = get_key_input();
            if (key == '1') 
            {
                mavros_msgs::CommandBool arm_cmd1;
                arm_cmd1.request.value = true;
                if (arming_client1.call(arm_cmd1) && arm_cmd1.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }

                mavros_msgs::CommandBool arm_cmd2;
                arm_cmd2.request.value = true;
                if (arming_client2.call(arm_cmd2) && arm_cmd2.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }

                mavros_msgs::CommandBool arm_cmd3;
                arm_cmd3.request.value = true;
                if (arming_client3.call(arm_cmd3) && arm_cmd3.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }

                mavros_msgs::CommandBool arm_cmd4;
                arm_cmd4.request.value = true;
                if (arming_client4.call(arm_cmd4) && arm_cmd4.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }

                mavros_msgs::CommandBool arm_cmd5;
                arm_cmd5.request.value = true;
                if (arming_client5.call(arm_cmd5) && arm_cmd5.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }

                mavros_msgs::CommandBool arm_cmd6;
                arm_cmd6.request.value = true;
                if (arming_client6.call(arm_cmd6) && arm_cmd6.response.success) 
                {
                    ROS_INFO("[✔] Arming succeeded");
                } 
                else 
                {
                    ROS_WARN("[!] Arming failed");
                }
            
            }

            else if (key == '2') 
            {
                mavros_msgs::SetMode mode_cmd1;
                mode_cmd1.request.custom_mode = "OFFBOARD";
                if (set_mode_client1.call(mode_cmd1) && mode_cmd1.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }

                mavros_msgs::SetMode mode_cmd2;
                mode_cmd2.request.custom_mode = "OFFBOARD";
                if (set_mode_client2.call(mode_cmd2) && mode_cmd2.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }

                mavros_msgs::SetMode mode_cmd3;
                mode_cmd3.request.custom_mode = "OFFBOARD";
                if (set_mode_client3.call(mode_cmd3) && mode_cmd3.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }

                mavros_msgs::SetMode mode_cmd4;
                mode_cmd4.request.custom_mode = "OFFBOARD";
                if (set_mode_client4.call(mode_cmd4) && mode_cmd4.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }

                mavros_msgs::SetMode mode_cmd5;
                mode_cmd5.request.custom_mode = "OFFBOARD";
                if (set_mode_client5.call(mode_cmd5) && mode_cmd5.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }

                mavros_msgs::SetMode mode_cmd6;
                mode_cmd6.request.custom_mode = "OFFBOARD";
                if (set_mode_client6.call(mode_cmd6) && mode_cmd6.response.mode_sent) 
                {
                    ROS_INFO("[✔] Offboard mode set");
                } 
                else 
                {
                    ROS_WARN("[!] Failed to set Offboard mode");
                }
            } 
            else 
            {
                ROS_INFO("Key '%c' not bound to action", key);
            }
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "wheelbird_mode_switcher");
    WheelbirdModeSwitcher node;
    node.spin();
    return 0;
}
