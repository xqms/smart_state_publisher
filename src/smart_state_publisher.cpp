// Efficient alternative to joint_state_publisher + robot_state_publisher
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

using JointState = sensor_msgs::JointState;

class Source
{
public:
    using Callback = std::function<void(const JointState&, const ros::Time&)>;

    explicit Source(const std::string& topic, const ros::Duration& minPeriod, const Callback& cb)
     : m_topic{topic}, m_minPeriod{minPeriod}, m_cb{cb}
    {
        subscribe();
    }

    Source(const Source&) = delete;
    Source(Source&& other)
     : m_topic{other.m_topic}, m_minPeriod{other.m_minPeriod}, m_cb{other.m_cb}
    {
        subscribe();
    }

    Source& operator=(const Source&) = delete;
    Source& operator=(Source&&) = delete;

private:
    void subscribe()
    {
        m_subscriber = ros::NodeHandle{}.subscribe(
            m_topic, 1,
            &Source::handleMsg, this,
            ros::TransportHints().udp()
        );
    }

    void handleMsg(const ros::MessageEvent<JointState>& event)
    {
        auto& js = *event.getMessage();
        if(js.header.stamp - m_lastStamp < m_minPeriod)
            return;

        m_cb(js, event.getReceiptTime());
        m_lastStamp = js.header.stamp;
    }

    std::string m_topic;
    ros::Duration m_minPeriod;

    ros::Subscriber m_subscriber;
    ros::Time m_lastStamp;
    Callback m_cb;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smart_state_publisher");

    ros::NodeHandle nh{"~"};

    bool diagnostics = false;
    nh.getParam("diagnostics", diagnostics);

    bool publishAllJoints = false;
    nh.getParam("publish_all_joints", publishAllJoints);

    robot_model_loader::RobotModelLoader loader("robot_description", false);
    auto model = loader.getModel();

    robot_state::RobotState state(model);
    state.setToDefaultValues();

    // Publish static transforms
    tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    {
        std::vector<geometry_msgs::TransformStamped> transforms;

        for(auto& linkModel : model->getLinkModels())
        {
            auto parent = linkModel->getParentLinkModel();
            if(!parent)
                continue;

            if(linkModel->getParentJointModel()->getType() == moveit::core::JointModel::FIXED)
            {
                while(parent->getParentLinkModel() && parent->getParentJointModel()->getType() == moveit::core::JointModel::FIXED)
                    parent = parent->getParentLinkModel();

                Eigen::Isometry3d transform = state.getGlobalLinkTransform(parent->getName()).inverse() * state.getGlobalLinkTransform(linkModel->getName());

                geometry_msgs::TransformStamped msg = tf2::eigenToTransform(transform);
                msg.header.frame_id = parent->getName();
                msg.header.stamp = ros::Time{0};
                msg.child_frame_id = linkModel->getName();

                transforms.push_back(std::move(msg));
            }
        }

        staticBroadcaster.sendTransform(transforms);
    }

    // Publish all received (rate-limited) messages on the agg topic
    ros::Publisher pub_js_agg = nh.advertise<sensor_msgs::JointState>("joint_states_agg", 10);

    // Dynamic transforms
    tf2_ros::TransformBroadcaster broadcaster;
    std::vector<double> msgLatency;
    std::vector<double> subscriberLatency;
    std::vector<double> processingLatency;


    auto update = [&](const JointState& msg, const ros::Time& receiptTime){
        ros::Time startTime = ros::Time::now();

        if(msg.name.size() != msg.position.size())
        {
            ROS_ERROR_THROTTLE(1.0, "Ignoring invalid joint_state msg");
            return;
        }

        pub_js_agg.publish(msg);

        std::vector<geometry_msgs::TransformStamped> transforms;

        try
        {
            state.setVariableValues(msg);
        }
        catch(moveit::Exception& e)
        {
            std::stringstream ss;

            ss << "Could net set variable values - probably a joint is not known. Known joints:";

            for(auto joint : state.getVariableNames())
                ss << " " << joint;

            ROS_ERROR("%s", ss.str().c_str());
            throw;
        }

        std::vector<bool> jointUpdated(model->getJointModels().size(), false);
        for(std::size_t j = 0; j < jointUpdated.size(); ++j)
            jointUpdated[j] = state.dirtyJointTransform(model->getJointModel(j));

        state.updateLinkTransforms();

        for(std::size_t j = 0; j < jointUpdated.size(); ++j)
        {
            auto joint = model->getJointModel(j);

            if(joint->getType() == robot_model::JointModel::FIXED)
                continue;

            if(!publishAllJoints && !jointUpdated[j])
                continue;

            if(!joint)
                continue;

            auto parentLink = joint->getParentLinkModel();

            if(!parentLink)
                continue;

            Eigen::Isometry3d transform = joint->getChildLinkModel()->getJointOriginTransform() * state.getJointTransform(joint);

            geometry_msgs::TransformStamped tmsg = tf2::eigenToTransform(transform);
            tmsg.header.frame_id = joint->getParentLinkModel()->getName();
            tmsg.header.stamp = msg.header.stamp;
            tmsg.child_frame_id = joint->getChildLinkModel()->getName();

            transforms.push_back(std::move(tmsg));
        }

        ros::Time sendTime = ros::Time::now();

        broadcaster.sendTransform(transforms);

        if(diagnostics)
        {
            msgLatency.push_back((receiptTime - msg.header.stamp).toSec());
            subscriberLatency.push_back((startTime - receiptTime).toSec());
            processingLatency.push_back((sendTime - startTime).toSec());
        }
    };

    ros::SteadyTimer statTimer;
    if(diagnostics)
    {
        statTimer = nh.createSteadyTimer(ros::WallDuration(5.0), boost::function<void(const ros::SteadyTimerEvent&)>([&](auto&){
            if(subscriberLatency.empty())
                return;

            auto calcMean = [](std::vector<double>& array){
                double sum = 0.0;
                for(auto& x : array)
                    sum += x;

                double mean = sum / array.size();

                array.clear();

                return mean;
            };

            std::size_t numMsgs = msgLatency.size();
            double msgLatencyMean = calcMean(msgLatency);
            double subscriberLatencyMean = calcMean(subscriberLatency);
            double processingLatencyMean = calcMean(processingLatency);

            ROS_INFO("Diagnostics: (%5lu messages): %.5fs msg, %.5fs subscriber, %.5fs processing", numMsgs, msgLatencyMean, subscriberLatencyMean, processingLatencyMean);
       }));
    }

    std::vector<Source> sources;

    XmlRpc::XmlRpcValue paramSources;
    if(nh.getParam("sources", paramSources))
    {
        if(paramSources.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("sources parameter needs to have type array");
            return 1;
        }

        for(int i = 0; i < paramSources.size(); ++i)
        {
            XmlRpc::XmlRpcValue paramSource = paramSources[i];

            if(paramSource.getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                std::string topic = paramSource;
                sources.emplace_back(topic, ros::Duration{0.0}, update);
            }
            else if(paramSource.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                std::string topic = paramSource["topic"];
                double rate = paramSource["rate"];

                ros::Duration minPeriod{(rate == 0.0) ? 0.0 : (1.0 / rate)};

                sources.emplace_back(topic, minPeriod, update);
            }
            else
            {
                ROS_FATAL("Got invalid type in topics array");
                return 1;
            }
        }
    }
    else
        sources.emplace_back("joint_states", ros::Duration{0.0}, update);

    ros::spin();

    return 0;
}
