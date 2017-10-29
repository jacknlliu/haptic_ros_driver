#ifndef HAPTIC_DEVICE_H__
#define HAPTIC_DEVICE_H__

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <boost/thread/thread.hpp>
#include <mutex>
#include <vector>


class HapticDevice
{
	public:
        HapticDevice(ros::NodeHandle& node, float loop_rate, bool set_force);
        virtual ~HapticDevice();

        void PublishHapticData();
        void RegisterCallback();

        void GetHapticDataRun();
		
        void SetForce(double x, double y, double z);

        void SetForceLimit(double x, double y, double z);
        void VerifyForceLimit(double input_force[], std::vector<double> & output);
        void ForceCallback(const geometry_msgs::Vector3::ConstPtr &data);

        void Start();

    protected:
       std::shared_ptr<boost::thread> dev_op_thread_;

	private:
       ros::NodeHandle nh_;
       ros::Rate loop_rate_;
       int device_count_;
       int dev_id_;
       bool set_force_;
       bool keep_alive_=false;
       bool device_enabled_ = false;
       std::mutex val_lock_;
       bool force_released_;
       bool button0_state_=false;
       bool button1_state_=false;

       ros::Publisher position_pub_;
       ros::Publisher button_state_pub_;
       ros::Subscriber force_sub_;
       double position_[3];

       std::string position_topic_;
       std::string buttons_topic_;
       std::string force_topic_;

       double force_x_limit_;
       double force_y_limit_;
       double force_z_limit_;
       std::vector<double> force_;

};

#endif
