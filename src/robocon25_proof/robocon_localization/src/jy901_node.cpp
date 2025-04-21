#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>

serial::Serial ser;
ros::Publisher imu_pub;
std::string frame_id;

// JY901B数据解析
void parseIMUData(const uint8_t *data) {
    sensor_msgs::Imu imu_msg;
    
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_id;
    
    // 解析加速度数据 (g)
    int16_t ax = (data[3] << 8) | data[2];
    int16_t ay = (data[5] << 8) | data[4];
    int16_t az = (data[7] << 8) | data[6];
    
    // 转换为m/s^2
    imu_msg.linear_acceleration.x = ax / 32768.0 * 16.0 * 9.8;
    imu_msg.linear_acceleration.y = ay / 32768.0 * 16.0 * 9.8;
    imu_msg.linear_acceleration.z = az / 32768.0 * 16.0 * 9.8;
    
    // 解析角速度数据 (deg/s)
    int16_t gx = (data[9] << 8) | data[8];
    int16_t gy = (data[11] << 8) | data[10];
    int16_t gz = (data[13] << 8) | data[12];
    
    // 转换为rad/s
    imu_msg.angular_velocity.x = gx / 32768.0 * 2000.0 * M_PI / 180.0;
    imu_msg.angular_velocity.y = gy / 32768.0 * 2000.0 * M_PI / 180.0;
    imu_msg.angular_velocity.z = gz / 32768.0 * 2000.0 * M_PI / 180.0;
    
    // 解析角度数据 (deg)
    int16_t roll = (data[15] << 8) | data[14];
    int16_t pitch = (data[17] << 8) | data[16];
    int16_t yaw = (data[19] << 8) | data[18];
    
    // 转换为四元数
    double roll_rad = roll / 32768.0 * M_PI;
    double pitch_rad = pitch / 32768.0 * M_PI;
    double yaw_rad = yaw / 32768.0 * M_PI;
    
    // 简单地使用欧拉角转四元数
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);
    
    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;
    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;
    
    // 添加协方差
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    
    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jy901_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std::string port;
    int baud, frequency;
    
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    private_nh.param<int>("baud", baud, 9600);
    private_nh.param<std::string>("frame_id", frame_id, "imu_link");
    private_nh.param<int>("frequency", frequency, 100);
    
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 50);
    
    try {
        ser.setPort(port);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port: " << port);
        return -1;
    }
    
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port " << port << " opened successfully.");
    } else {
        return -1;
    }
    
    ros::Rate loop_rate(frequency);
    uint8_t buffer[50];
    
    while (ros::ok()) {
        if (ser.available()) {
            ser.read(buffer, 22); // JY901B数据包长度通常为11字节
            if (buffer[0] == 0x55) { // 检查帧头
                parseIMUData(buffer);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}