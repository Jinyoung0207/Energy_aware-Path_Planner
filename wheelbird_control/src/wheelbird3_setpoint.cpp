#include "multi_setpoint_ctrl.hpp"

void multi_setpoint_ctrl::pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    UAV1_pose = *msg;
}

void multi_setpoint_ctrl::controller_mode_num_Callback(const std_msgs::Int64::ConstPtr &msg)
{
    controller_mode_num=*msg;
}

void multi_setpoint_ctrl::initial_pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    UAV1_initial_pose = *msg;
}

void multi_setpoint_ctrl::ref_1_Callback(const nav_msgs::Path::ConstPtr &msg)
{
    buf1.lock();
    ref_1_queue.push(*msg);
    buf1.unlock();
}
void multi_setpoint_ctrl::state1Callback(const mavros_msgs::State::ConstPtr& msg) 
{
    current_state1 = *msg;
}

void multi_setpoint_ctrl::mode_num_Callback(const std_msgs::Int64::ConstPtr &msg)
{
    mode_num=*msg;
}
void multi_setpoint_ctrl::th_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    th=*msg;
}
void multi_setpoint_ctrl::land_vel_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    land_vel=*msg;
}

void multi_setpoint_ctrl::ref_path_update()
{
    if(!ref_1_queue.empty())
    {
        buf1.lock();
        ref_path_1_msg = ref_1_queue.front();
        ref_1_queue.pop();
        ref_traj_1_in = true;
        buf1.unlock();
    }
}

void rotate_point(double x, double y, double a, double result[2])
{
    result[0] = x*cos(a*M_PI/180) - y*sin(a*M_PI/180);
    result[1] = x*sin(a*M_PI/180) + y*cos(a*M_PI/180);
}

void multi_setpoint_ctrl::averaging_pose()
{
    avp1.pose.position.x=0;
    avp1.pose.position.y=0;
    avp1.pose.position.z=0;

    avp1.pose.orientation.x=0;
    avp1.pose.orientation.y=0;
    avp1.pose.orientation.z=0;
    avp1.pose.orientation.w=0;

    for(int i=0;i<100;++i)
    {
        avp1.pose.position.x+=UAV1_pose.pose.position.x;
        avp1.pose.position.y+=UAV1_pose.pose.position.y;
        avp1.pose.position.z+=UAV1_pose.pose.position.z;

        avp1.pose.orientation.x+=UAV1_pose.pose.orientation.x;
        avp1.pose.orientation.y+=UAV1_pose.pose.orientation.y;
        avp1.pose.orientation.z+=UAV1_pose.pose.orientation.z;
        avp1.pose.orientation.w+=UAV1_pose.pose.orientation.w;  
    } 
    avp1.pose.position.x=avp1.pose.position.x/100;
    avp1.pose.position.y=avp1.pose.position.y/100;
    avp1.pose.position.z=avp1.pose.position.z/100;

    avp1.pose.orientation.x=avp1.pose.orientation.x/100;
    avp1.pose.orientation.y=avp1.pose.orientation.y/100;
    avp1.pose.orientation.z=avp1.pose.orientation.z/100;
    avp1.pose.orientation.w=avp1.pose.orientation.w/100;

    avp1_done=1;
}

double multi_setpoint_ctrl::yaw_from_quaternion(double x, double y, double z, double w)
{
    // 요 (z축 회전)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void multi_setpoint_ctrl::set_initial_orientation()
{
    // initial_yaw_degree = yaw_from_quaternion(avp1.pose.orientation.x,avp1.pose.orientation.y, avp1.pose.orientation.z, avp1.pose.orientation.w) * 180 / M_PI;
    initial_yaw_degree=0;
    rotate_point(UAV1_initial_pose.pose.position.x, UAV1_initial_pose.pose.position.y, initial_yaw_degree, UAV1_init);
}

void multi_setpoint_ctrl::set_point()
{
    if(ref_traj_1_in)
    {
        // error1=sqrt(pow(sp1.pose.position.x-UAV1_pose.pose.position.x,2)+pow(sp1.pose.position.y-UAV1_pose.pose.position.y,2)+pow(sp1.pose.position.z-UAV1_pose.pose.position.z,2));
        std::cout<<"::::: ref_traj_check done :::::"<<std::endl;

        sp1.pose.position.x = avp1.pose.position.x + ref_path_1_msg.poses[9].pose.position.x - UAV1_init[0];
        sp1.pose.position.y = avp1.pose.position.y + ref_path_1_msg.poses[9].pose.position.y - UAV1_init[1];
        sp1.pose.position.z = avp1.pose.position.z + ref_path_1_msg.poses[9].pose.position.z;
        // sp1.pose.position.z = avp1.pose.position.z + 2;
        sp1.pose.orientation = ref_path_1_msg.poses[9].pose.orientation;
        

        error1=sqrt(pow(sp1.pose.position.x-UAV1_pose.pose.position.x,2)+pow(sp1.pose.position.y-UAV1_pose.pose.position.y,2)+pow(sp1.pose.position.z-UAV1_pose.pose.position.z,2));
        ROS_INFO_STREAM("UAV1 error: " << error1 );

    }

}

void multi_setpoint_ctrl::publishAll()
{

    sp1.header.frame_id = "map";
    sp1.header.stamp = ros::Time::now();

    set_point_1_pub.publish(sp1);

    //Rviz path
    UAV1_path.header.frame_id="map";
    UAV1_path.header.stamp=ros::Time::now();
    path_1.pose.position.x = UAV1_pose.pose.position.x - avp1.pose.position.x +  UAV1_init[0];
    path_1.pose.position.y = UAV1_pose.pose.position.y - avp1.pose.position.y +  UAV1_init[1];
    path_1.pose.position.z = UAV1_pose.pose.position.z - avp1.pose.position.z;
    UAV1_path.poses.emplace_back(path_1);
    path_1_pub.publish(UAV1_path); 

    //Rviz point
    UAV1_point.header.frame_id="map";
    UAV1_point.header.stamp=ros::Time::now();
    UAV1_point.point.x= UAV1_pose.pose.position.x - avp1.pose.position.x +  UAV1_init[0];
    UAV1_point.point.y= UAV1_pose.pose.position.y - avp1.pose.position.y +  UAV1_init[1];
    UAV1_point.point.z= UAV1_pose.pose.position.z - avp1.pose.position.z;
    pose_point_1_pub.publish(UAV1_point);  
    
    if(avp1_done==1)
    {
        avp1_pub.publish(avp1);
        initial_yaw.data=initial_yaw_degree;
        initial_yaw_1_pub.publish(initial_yaw);
    }
    
    if(ref_traj_1_in)
    {
        x_error = UAV1_pose.pose.position.x - ref_path_1_msg.poses[0].pose.position.x;
        y_error = UAV1_pose.pose.position.y - ref_path_1_msg.poses[0].pose.position.y;
        z_error = UAV1_pose.pose.position.z - ref_path_1_msg.poses[0].pose.position.z;
        
        squared_error = x_error * x_error + y_error * y_error + z_error * z_error;
        
        cumulative_squared_error += squared_error;
        
        sample_count++;
        
        rmse = std::sqrt(cumulative_squared_error / sample_count);
    
        std::cout << "rmse result is : " << rmse << "sample_count_is: " << sample_count << "cumulative_squared_error_is: " << cumulative_squared_error << std::endl;
        x_error = 0;
        y_error = 0;
        z_error = 0;
    }
}


void multi_setpoint_ctrl:: init()
{
    mode_flag_sub = nh.subscribe<std_msgs::Int64>("/wheelbird3/mode_flag", 1, &multi_setpoint_ctrl::controller_mode_num_Callback, this);
    
    current_pose_1_sub = nh.subscribe<geometry_msgs::PoseStamped>("wheelbird3/mavros/local_position/pose", 1, &multi_setpoint_ctrl::pose1Callback, this);
    ref_1_sub = nh.subscribe<nav_msgs::Path>("/wheelbird3/ref_trajectory", 1, &multi_setpoint_ctrl::ref_1_Callback, this);
    initial_pose1_sub = nh.subscribe<geometry_msgs::PoseStamped>("UAV1/initial_pose", 1, &multi_setpoint_ctrl::initial_pose1Callback, this);
    set_mode_sub=nh.subscribe<std_msgs::Int64>("/mode_num", 1, &multi_setpoint_ctrl::mode_num_Callback, this);
    state1_sub = nh.subscribe<mavros_msgs::State>("wheelbird3/mavros/state", 1, &multi_setpoint_ctrl::state1Callback, this);

    th_sub=nh.subscribe<std_msgs::Float32>("/th", 1, &multi_setpoint_ctrl::th_Callback, this);
    land_vel_sub=nh.subscribe<std_msgs::Float32>("/land_vel", 1, &multi_setpoint_ctrl::land_vel_Callback, this);
    param_set_client1 = nh.serviceClient<mavros_msgs::ParamSet>("wheelbird3/mavros/param/set");// Auto landing speed
    param_th_client1 = nh.serviceClient<mavros_msgs::ParamSet>("wheelbird3/mavros/param/set"); // Auto landing vertical velocity threshold

    set_point_1_pub = nh.advertise<geometry_msgs::PoseStamped>("/wheelbird3/mavros/setpoint_position/local", 1);
    pose_point_1_pub = nh.advertise<geometry_msgs::PointStamped>("wheelbird3/pose_point", 1);
    path_1_pub = nh.advertise<nav_msgs::Path>("UAV1/path", 1); 
    error1_pub= nh.advertise<std_msgs::Float32>("/UAV1/error",1);
    avp1_pub = nh.advertise<geometry_msgs::PoseStamped>("/UAV1/avp",1);
    initial_yaw_1_pub=nh.advertise<std_msgs::Float32>("/initial_yaw_degree_1",1);
}
void multi_setpoint_ctrl::run()
{
    UAV1_initial_pose.pose.position.x = 0;
    UAV1_initial_pose.pose.position.y = 0;
    UAV1_initial_pose.pose.position.z = 0;
    set_initial_orientation();
    if(wait_time1==30)
    {
        averaging_pose();    
        wait_time1+=1;  
    }
    else if(wait_time1>30)
    {
        ref_path_update();
        std::cout<<"-----------------------------------------------"<<std::endl;
        std::cout << "global path in ... " <<  std::endl;
        std::cout<<"UAV1 current mode :"<<current_state1.mode<<std::endl;
        set_point();
    }
    else if(wait_time1<30)
    {
        wait_time1+=1;
    }

    if(controller_mode_num.data==1)
    {
        publishAll();
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "wheelbird3_setpoint_node");
    ros::NodeHandle nh;

    multi_setpoint_ctrl setpoint_ctrl(nh);
    ros::Rate loop_rate(10); 

    // Main loop
    setpoint_ctrl.init();
    while (ros::ok()) 
    {
        setpoint_ctrl.run();
        ros::spinOnce();  
        loop_rate.sleep();  
    }

    return 0;
}