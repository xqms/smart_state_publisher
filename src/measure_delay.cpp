// Measure delay of a particular tf transform
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

using Msg = ros::MessageEvent<const tf2_msgs::TFMessage>;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "measure_delay");
    ros::NodeHandle nh{"~"};

    bool udp = false;
    nh.getParam("udp", udp);

    std::string frame;
    if(!nh.getParam("frame", frame))
    {
        ROS_FATAL("Need frame parameter");
        return 1;
    }

    std::vector<double> delays;

    ros::TransportHints hints = ros::TransportHints{}.tcpNoDelay(true);
    if(udp)
        hints = hints.udp();

    ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, boost::function<void(const Msg&)>([&](const Msg& msgEvent){
        auto msg = msgEvent.getMessage();
        for(auto& trans : msg->transforms)
        {
            if(trans.child_frame_id == frame)
            {
                delays.push_back((msgEvent.getReceiptTime() - trans.header.stamp).toSec());
                break;
            }
        }
    }), {}, hints);

    ros::SteadyTimer timer = nh.createSteadyTimer(ros::WallDuration(2.0), boost::function<void(const ros::SteadyTimerEvent&)>([&](const auto&){
        if(delays.empty())
        {
            ROS_WARN("No transforms yet...");
            return;
        }

        double sum = 0.0;
        for(auto& delay : delays)
            sum += delay;

        double mean = sum / delays.size();

        ROS_INFO("Mean delay (%5lu messages): %f", delays.size(), mean);
        delays.clear();
    }));

    ros::spin();

    return 0;
}
