#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <string>                   
#include <iostream>                 
#include <serial/serial.h>          
#include <vector>                   
#include <sstream>                  
#include <exception>                
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/constants/constants.hpp>
//#include <chiki-briki_i_v_damki.h>


#include <dynamic_reconfigure/server.h>
#include <mcu_interface/parametersConfig.h>
#include "mcu_interface/AlignMcuCamPhase.h"

std::string IMU_TOPIC = "mcu_imu";
std::string IMU_TEMP_TOPIC_POSTFIX = "_temp";
std::string CAMERAS_TS_TOPIC = "cameras_ts";
std::string LIDAR_TS_TOPIC = "lidar_ts";

std::string IMU_TEMP_TOPIC = IMU_TOPIC + IMU_TEMP_TOPIC_POSTFIX;
std::string PORT;// = "/dev/ttyUSB200";  // port name
const int BAUD = 500000;            
int RATE = 10000;                   
int TIMOUT = 10;                    
uint32_t FRACT_NUMBER = 25600000; 
double G = 9.81;
const double PI = boost::math::constants::pi<double>();
int TEMP_BUF_SIZE = 200;
int NUM_OF_IMUS = 1;
int NUM_OF_TS_FIELDS = 4;
int NUM_OF_IMU_FIELDS = 7;
int PLD_STRT_INDX = 2; // payload starting index in received string line

uint32_t CAMERAS_TIM_FRACT_NUMBER = 7680000; 
uint32_t alignment_subs = 0;
ros::Time last_cameras_ts = ros::Time(0);
uint8_t CAMERAS_FRAME_RATE = 5;

class FieldsCount{
  public:
    int count;   
    FieldsCount (int start = 0) {
        count = start;
    }
    void add (int additive) {
        count += additive;
    }
    int current (void) {
        return count;
    }
};

std::vector<int16_t> string_to_ints(std::string str, int start_from = 0) { 
    std::stringstream ss;    
    /* Storing the whole string into string stream */
    ss << str.substr(start_from); 
    /* Running loop till the end of the stream */
    std::string temp; 
    int found; 
    
    std::vector<int16_t> ints;        

    while (!ss.eof()) { 
        /* extracting word by word from stream */
        ss >> temp; 
        /* Checking the given word is integer or not */
        if (std::stringstream(temp) >> std::hex >> found) {
            ints.push_back(static_cast<int16_t>(found));
        }
        temp = ""; 
    } 
    return ints;
} 

std::vector<int16_t> subvector(std::vector<int16_t> const &initial_v, int starting_index) {
   std::vector<int16_t> sub_v(initial_v.begin() + starting_index, initial_v.end());
   return sub_v;
}

ros::Time ints_to_board_ts(std::vector<int16_t> input_ints, FieldsCount * fc_pointer, int first_element=0) {
	std::vector<int16_t> ints = subvector(input_ints, fc_pointer->current());
	
	double secs = static_cast<uint16_t>(ints[0]) * 60.0 + static_cast<uint16_t>(ints[1]) * 1.0 + static_cast<uint32_t>(ints[2]<<16 | static_cast<uint16_t>(ints[3]))*1.0/FRACT_NUMBER;
	ros::Time board_ts = ros::Time(secs);

    fc_pointer->add(NUM_OF_TS_FIELDS);
    return board_ts;
}

boost::numeric::ublas::vector<double> ints_to_imu_meas(std::vector<int16_t> input_ints, FieldsCount * fc_pointer, int first_element=0) {
    std::vector<int16_t> ints = subvector(input_ints, fc_pointer->current());
    boost::numeric::ublas::vector<double> imu_meas(NUM_OF_IMU_FIELDS);
    for (int i = 0; i < imu_meas.size(); i++) {
        // acc
        if (i < 3) {
            imu_meas(i) = ints[i] / 16384.0 * G;
        }
        // temperature
        else if (i == 3) {
            imu_meas(i) = ints[3] / 340.0 + 35.0;
        }
        // gyro
        else if (i >= 3) {
            imu_meas(i) =  ints[i] / 131.0 / 180.0 * PI;
        }
    }

    fc_pointer->add(NUM_OF_IMU_FIELDS);
    return imu_meas;
}

void publish_imu(ros::Publisher pub, uint8_t imu_n, ros::Time ts, boost::numeric::ublas::vector<double> imu_meas) {
    // publish_imu data [a in m/s^2] and [w in rad/s]
    std::string frame_id = IMU_TOPIC + std::to_string(imu_n);
    sensor_msgs::Imu msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    // linear_acceleration
    msg.linear_acceleration.x = imu_meas[0];
    msg.linear_acceleration.y = imu_meas[1];
    msg.linear_acceleration.z = imu_meas[2];
    // angular_velocity
    msg.angular_velocity.x = imu_meas[4];
    msg.angular_velocity.y = imu_meas[5];
    msg.angular_velocity.z = imu_meas[6];
    // Publish the message.
    pub.publish(msg);
}

void publish_imu_temperature(ros::Publisher pub, uint8_t imu_n, ros::Time ts, boost::numeric::ublas::vector<double> imu_meas) {
    std::string frame_id = IMU_TOPIC + std::to_string(imu_n) + IMU_TEMP_TOPIC_POSTFIX;
    sensor_msgs::Temperature msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    msg.temperature = imu_meas[3];
    
    pub.publish(msg);
}

void publish_cameras_ts(ros::Publisher pub, ros::Time ts) {
    std::string frame_id = CAMERAS_TS_TOPIC;
    sensor_msgs::TimeReference msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    msg.time_ref = ros::Time::now();
    pub.publish(msg);
}

void publish_lidar_ts(ros::Publisher pub, ros::Time ts) {
    std::string frame_id = LIDAR_TS_TOPIC;
    sensor_msgs::TimeReference msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    msg.time_ref = ros::Time::now();
    pub.publish(msg);
}

void pub_distributer(std::string str) {
}


// dynamic reconfigure callback
void dynamic_reconfigure_callback(mcu_interface::parametersConfig &config, uint32_t level)
{
    if (config.start_sync)
    {
        // Marsel, here goes the sync function/code only when set to True
        ROS_WARN("dynamic_reconfigure_callback: Flag set to True");//TODO change/remove this
    }
}

// Service server
bool align_phase(mcu_interface::AlignMcuCamPhase::Request  &req,
         mcu_interface::AlignMcuCamPhase::Response &res, serial::Serial *serial)
{
    double phase = req.a;
    ROS_WARN("phase %f", phase);
    phase = phase - last_cameras_ts.toSec();
    ROS_WARN("last_cameras_ts.toSec() %f", last_cameras_ts.toSec());
    ROS_WARN("phase = phase - last_ts %f", phase);
    //phase = phase * CAMERAS_FRAME_RATE; * CAMERAS_TIM_FRACT_NUMBER;
    phase = std::fmod(phase, 1.0 / CAMERAS_FRAME_RATE);
    ROS_WARN("phase=std::fmod(phase,1/CAMERAS_FRAME_RATE) %f", phase);
    
    if (phase < 0) {
        phase += 1.0 / CAMERAS_FRAME_RATE;
    }
    ROS_WARN("phase_=std::fmod(phase,1/CAMERAS_FRAME_RATE) %f", phase);
    phase = phase * CAMERAS_TIM_FRACT_NUMBER;
    ROS_WARN("phase=phase*CAMERAS_TIM_FRACT_NUMBER %f", phase);
    
    alignment_subs = static_cast<uint32_t> (std::round(phase));
    ROS_WARN("alignment_subs %d\n", alignment_subs);


    serial->write((uint8_t *)&alignment_subs, 4);
    // Stub
    res.response = "done";
    //ROS_WARN("sending back response: [%s]", res.response.c_str());
    return true;
}

int main(int argc, char **argv) {
    // Register signal and signal handler
    if (argc < 2) {
        std::cout << "Please, specify serial device.For example, \"/dev/ttyUSB0\"" << std::endl;
        return 0;
    }
    
    PORT = argv[1];
    bool board_starting_ts_is_read = false;
    std::string str;
    
    ros::Time sys_starting_ts, board_starting_ts, ts, ts_old;
    ros::Duration delta_ts;
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "mcu_interface");
    

    // Create a publisher object.
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(IMU_TOPIC, RATE);
	ros::Publisher imu_temp_pub = nh.advertise<sensor_msgs::Temperature>(IMU_TEMP_TOPIC, RATE);
	ros::Publisher cameras_ts_pub = nh.advertise<sensor_msgs::TimeReference>(CAMERAS_TS_TOPIC, RATE);
	ros::Publisher lidar_ts_pub = nh.advertise<sensor_msgs::TimeReference>(LIDAR_TS_TOPIC, RATE);
	
	// Configure dynamic reconfigure
	dynamic_reconfigure::Server<mcu_interface::parametersConfig> server;
    dynamic_reconfigure::Server<mcu_interface::parametersConfig>::CallbackType f;
    f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
    server.setCallback(f);
	
	
    // open port, baudrate, timeout in milliseconds
    serial::Serial serial(PORT, BAUD, serial::Timeout::simpleTimeout(TIMOUT));
    
    
    // Service configure
    ros::ServiceServer service = nh.advertiseService<mcu_interface::AlignMcuCamPhase::Request, mcu_interface::AlignMcuCamPhase::Response>("align_mcu_cam_phase", boost::bind(align_phase, _1, _2, &serial));


    // check if serial port open
    std::cout << "Serial port is...";
    if(serial.isOpen())
        std::cout << " open." << std::endl;
    else
        std::cout << " not open!" << std::endl;

    // Clean from possibly broken string
    while(serial.available() < TEMP_BUF_SIZE) {
        str = serial.readline(); 
        if(str.at(str.size()-1)=='\n') {
            break;
        }
    }

    //serial.write((uint8_t *)&alignment_subs, 4);

    // Main loop
    while(ros::ok()) {
        if(serial.available() > TEMP_BUF_SIZE) {
            str = serial.readline();
            //std::cout << str << std::endl;
            std::vector<int16_t> ints = string_to_ints(str, PLD_STRT_INDX);
            FieldsCount fields_count;
            ros::Time ts = ints_to_board_ts(ints, &fields_count);
		   	switch (str.at(0)) {
				case 'i': {
					boost::numeric::ublas::vector<double> imu_meas;
					imu_meas = ints_to_imu_meas(ints, &fields_count);
					publish_imu(imu_pub, 0, ts, imu_meas);
					publish_imu_temperature(imu_temp_pub, 0, ts, imu_meas);
					break;
				}
				case 'c': {
					publish_cameras_ts(cameras_ts_pub, ts);
                    last_cameras_ts = ts;
					break;
				}
				case 'l': {
					publish_lidar_ts(lidar_ts_pub, ts);
					//std::cout << ts << std::endl;
				}
			}
			//serial.write((uint8_t *)&alignment_subs, 4);
			ros::spinOnce();// this checks for callbacks (dynamic reconfigure)
	    }
	    usleep(100);
	}
}
