// Interface to the Microstrain 3DM-GX3-25
// N. Michael
// y = -y
// x = -z
// z = -x

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <thread>

using namespace std;

typedef boost::asio::serial_port_base sb;

#define REPLY_LENGTH 4
#define GRAVITY 9.80665

boost::asio::serial_port* serial_port = nullptr;
const char stop[3] = {'\xFA','\x75','\xB4'};
char mode[4] = {'\xD4','\xA3','\x47','\x00'};
unsigned char reply[REPLY_LENGTH];
std::string name;

static float extract_float(unsigned char* addr) {
    float tmp;
    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr + 1);
    *((unsigned char*)(&tmp) + 1) = *(addr + 2);
    *((unsigned char*)(&tmp)) = *(addr + 3);
    return tmp;
}

static int extract_int(unsigned char* addr) {
    int tmp;
    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr + 1);
    *((unsigned char*)(&tmp) + 1) = *(addr + 2);
    *((unsigned char*)(&tmp)) = *(addr + 3);
    return tmp;
}

bool validate_checksum(const unsigned char* data, unsigned short length) {
    unsigned short chksum = 0;
    unsigned short rchksum = 0;
    for (unsigned short i = 0; i < length - 2; i++)
        chksum += data[i];
    rchksum = data[length - 2] << 8;
    rchksum += data[length - 1];
    return (chksum == rchksum);
}

void makeUnsignedInt16(unsigned int val, unsigned char* high, unsigned char* low) {
    *low = static_cast<unsigned char>(val);
    *high = static_cast<unsigned char>(val >> 8);
}

void makeUnsignedInt32(int val, unsigned char* byte3, unsigned char* byte2, unsigned char* byte1, unsigned char* byte0)
{
    *byte0 = static_cast<unsigned char>(val);
    *byte1 = static_cast<unsigned char>(val >> 8);
    *byte2 = static_cast<unsigned char>(val >> 16);
    *byte3 = static_cast<unsigned char>(val >> 24);
}

void check_port(boost::asio::serial_port* serial_port, const std::string& port) {
    try {
        serial_port->open(port);
    } catch (boost::system::system_error& error) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "%s: Failed to open port %s with error %s", name.c_str(), port.c_str(), error.what());
        rclcpp::shutdown();
    }

    if (!serial_port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "%s: failed to open serial port %s", name.c_str(), port.c_str());
        rclcpp::shutdown();
    }
}


void set_data_rate(boost::asio::serial_port* serial_port, int decimation)
{

  unsigned char ocsb;       // orientation, coning & sculling byte
  unsigned char decu, decl; // decimation value upper & lower bytes
  unsigned char wndb;       // digital filter window size byte
  
  makeUnsignedInt16(decimation, &decu, &decl);

  int filter_window_size = 15;
  
  ocsb = 0x03;

  wndb = static_cast<unsigned char>(filter_window_size);

  // Set the mode to continous output
  unsigned char set_sampling_params_string[] = {
        0xDB, // Byte  1    : command
        0xA8,                      // Bytes 2-3  : confirm intent
        0xB9,
        0x01, // Byte  4    : change params
        decu, // Bytes 5-6  : decimation value
        decl,
        0x00, // Bytes 7-8  : flags - orient
        ocsb,
        wndb, // Byte  9    : gyro/accel window size
        0x11, // Byte  10   : magneto window size
        0x00, // Byte  11-12: up compensation
        0x0A,
        0x00, // Byte  13-14: north compensation
        0x0A,
        0x01, // Byte  15   : low magneto power
        0x00, // Bytes 16-20: reserved (zeros)
        0x00, 0x00, 0x00, 0x00
    };
  unsigned char reply_data[19];
  boost::asio::write(*serial_port, boost::asio::buffer(set_sampling_params_string, 20));
  boost::asio::read(*serial_port, boost::asio::buffer(reply_data, 19));
  
  if (!validate_checksum(reply_data, 19))
    {
    RCLCPP_ERROR(rclcpp::get_logger("LoggerName"), "%s: failed to set mode to continuous output", name.c_str());      if (serial_port->is_open())
        serial_port->close();
      exit(EXIT_FAILURE); // 프로그램 종료
    }
}

void set_baudrate(boost::asio::serial_port* serial_port, int baud)
{

  unsigned char baud0, baud1, baud2, baud3;

  // Convert our int baud rate, into 4 seperate bytes
  makeUnsignedInt32(baud, &baud3, &baud2, &baud1, &baud0);

  // Set the mode to continous output
  unsigned char set_comms_baud_rate_string[] = {
        0xD9, // Byte  1    : command
        0xC3, // Bytes 2-3: confirm intent
        0x55,
        0x01,  // Byte  4  : port selector
        0x00,  // Byte  5  : temporary change
        baud3, // Bytes 6-9: baud rate
        baud2,
        baud1,
        baud0,
        0x10, // Byte  10 : port config
        0x00  // Byte  11 : reserved (zero)
    };
    
  unsigned char reply_baud[10];
  boost::asio::write(*serial_port, boost::asio::buffer(set_comms_baud_rate_string, 11));
  boost::asio::read(*serial_port, boost::asio::buffer(reply_baud, 10));
  
  if (!validate_checksum(reply_baud, 10))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LoggerName"), "%s: failed to set baudrate", name.c_str());
      if (serial_port->is_open())
        serial_port->close();
      exit(EXIT_FAILURE); // 프로그램 종료
    }
}

void set_continous_output_mode(boost::asio::serial_port* serial_port)
{
  // Set the mode to continous output
  mode[3] = '\x02';
  boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
  boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
  if (!validate_checksum(reply, REPLY_LENGTH))
    {
      if (serial_port->is_open())
        serial_port->close();
      exit(EXIT_FAILURE); // 프로그램 종료
    }
} 
 
void set_timer(boost::asio::serial_port* serial_port)
{
  // Set Timer
  char set_timer[8] = {'\xD7','\xC1','\x29','\x01','\x00','\x00','\x00','\x00'};
  unsigned char reply_timer[7];
  boost::asio::write(*serial_port, boost::asio::buffer(set_timer, 8));
  // ros::Time t0 = ros::Time::now();  
  boost::asio::read(*serial_port, boost::asio::buffer(reply_timer, 7));
}

void set_stop_continous_mode(boost::asio::serial_port* serial_port)
{
  // Stop continous mode if it is running
  boost::asio::write(*serial_port, boost::asio::buffer(stop, 3));
  RCLCPP_WARN(rclcpp::get_logger("LoggerName"),"Wait 0.1s"); 
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1초 대기
}

void set_continous_preset_mode(boost::asio::serial_port* serial_port)
{
  // Set the continous preset mode
  const char preset[4] = {'\xD6','\xC6','\x6B','\xCC'};
  boost::asio::write(*serial_port, boost::asio::buffer(preset, 4));

  boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
  if (!validate_checksum(reply, REPLY_LENGTH))
    {
      RCLCPP_WARN(rclcpp::get_logger("LoggerName"), name.c_str());
      if (serial_port->is_open())
        serial_port->close();
      exit(EXIT_FAILURE); // 프로그램 종료
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("imu_3dm_gx3", options);
    name = node->get_name();

    int decimation;
    std::string port;
    int baud;
    std::string frame_id;
    double delay;

    node->declare_parameter("decimation", 3);
    node->declare_parameter("port", "/dev/ttyACM0");
    node->declare_parameter("baud", 230400);
    node->declare_parameter("frame_id", "world");
    node->declare_parameter("delay", 0.0);

    node->get_parameter("decimation", decimation);
    node->get_parameter("port", port);
    node->get_parameter("baud", baud);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("delay", delay);

    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);

    check_port(serial_port, port);

    //set_baudrate(serial_port, baud);

    sb::baud_rate baud_option(baud);
    sb::flow_control flow_control(sb::flow_control::none);
    sb::parity parity(sb::parity::none);
    sb::stop_bits stop_bits(sb::stop_bits::one);

    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);

    set_stop_continous_mode(serial_port);

    // Check the mode
    bool reInitFlag = false;
    boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
    boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
    
    if (!validate_checksum(reply, REPLY_LENGTH))
        {
        RCLCPP_ERROR(rclcpp::get_logger("LoggerName"),"%s: failed to get mode", name.c_str());
        if (serial_port->is_open())
            serial_port->close();
        reInitFlag = true;
        }

    if (reInitFlag)
    {
        RCLCPP_WARN(rclcpp::get_logger("LoggerName"),"In Re-Init");
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1초 대기

        check_port(serial_port, port);

        serial_port->set_option(baud_option);
        serial_port->set_option(flow_control);
        serial_port->set_option(parity);
        serial_port->set_option(stop_bits);

        // Check the mode
        boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
        boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
        if (!validate_checksum(reply, REPLY_LENGTH))
        {
            RCLCPP_ERROR(rclcpp::get_logger("LoggerName"),"%s: failed to get mode", name.c_str());
            if (serial_port->is_open())
            serial_port->close();
            return -1;
        }    
    }

    // If we are not in active mode, change it
    if (reply[2] != '\x01')
        {
        mode[3] = '\x01';
        boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
        boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
        if (!validate_checksum(reply, REPLY_LENGTH))
            {
            RCLCPP_ERROR(rclcpp::get_logger("LoggerName"),"%s: failed to set mode to active", name.c_str());
            if (serial_port->is_open())
                serial_port->close();
            return -1;
            }
        }

    set_continous_preset_mode(serial_port);
    set_data_rate(serial_port, decimation);
    set_continous_output_mode(serial_port);  
    set_timer(serial_port);
    
    RCLCPP_WARN(rclcpp::get_logger("LoggerName"),"Streaming Data...");

    unsigned short data_length = 79;
    unsigned char data[data_length];

    sensor_msgs::msg::Imu msg;
    sensor_msgs::msg::MagneticField msg_mag;

    // Create publishers
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    auto pub_mag = node->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

    // Main loop
    rclcpp::Rate rate(100); // Set loop rate
    
    rclcpp::Time t0 = node->now(); // ROS2 코드로 변경

    while (rclcpp::ok()) {
        
        boost::asio::read(*serial_port, boost::asio::buffer(data, data_length));

        if (!validate_checksum(data, data_length))
            {
            RCLCPP_ERROR(rclcpp::get_logger("LoggerName"),"%s: checksum failed on message", name.c_str());
            continue;
            }

        unsigned int k = 1;
        float acc[3];
        float ang_vel[3];
        float mag[3];
        float M[9];
        double T;
        for (unsigned int i = 0; i < 3; i++, k += 4)
            acc[i] = extract_float(&(data[k]));
        for (unsigned int i = 0; i < 3; i++, k += 4)
            ang_vel[i] = extract_float(&(data[k]));
        for (unsigned int i = 0; i < 3; i++, k += 4)
            mag[i] = extract_float(&(data[k]));
        for (unsigned int i = 0; i < 9; i++, k += 4)
            M[i] = extract_float(&(data[k]));
        T = extract_int(&(data[k])) / 62500.0;

        msg.header.stamp = t0 + rclcpp::Duration::from_seconds(T) - rclcpp::Duration::from_seconds(delay);        
        msg.header.frame_id = frame_id;
        msg.angular_velocity.x = ang_vel[0];
        msg.angular_velocity.y = ang_vel[1];
        msg.angular_velocity.z = ang_vel[2];
        msg.linear_acceleration.x = acc[0] * GRAVITY;
        msg.linear_acceleration.y = acc[1] * GRAVITY;
        msg.linear_acceleration.z = acc[2] * GRAVITY;
        // mat R(3,3);
        // for (unsigned int i = 0; i < 3; i++)
        //   for (unsigned int j = 0; j < 3; j++)
        //     R(i,j) = M[j*3+i];
        // colvec q = R_to_quaternion(R);
        Eigen::Matrix3d R;
        for (unsigned int i = 0; i < 3; i++)
            for (unsigned int j = 0; j < 3; j++)
            R(i,j) = M[j*3+i];
        Eigen::Quaternion<double> q(R);
        msg.orientation.w = (double)q.w();// q(0);
        msg.orientation.x = (double)q.x();// q(1);
        msg.orientation.y = (double)q.y();// q(2);
        msg.orientation.z = (double)q.z();// q(3);
        msg_mag.magnetic_field.x = -mag[2];
        msg_mag.magnetic_field.y = -mag[1];
        msg_mag.magnetic_field.z = -mag[0];
        msg_mag.header.stamp=msg.header.stamp;
        msg_mag.header.frame_id=msg.header.frame_id;
        pub->publish(msg);
        pub_mag->publish(msg_mag);
        
        //rate.sleep(); // Sleep to maintain loop rate
    }

    // Stop continuous mode and close device
    boost::asio::write(*serial_port, boost::asio::buffer(stop, 3));
    RCLCPP_WARN(rclcpp::get_logger("SerialPort"), "Wait 0.1s");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_port->close();

    rclcpp::shutdown();
    return 0;
}
