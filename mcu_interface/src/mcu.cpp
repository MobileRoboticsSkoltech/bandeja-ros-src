#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <string>                   
#include <iostream>                 
#include <fstream>
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
#include "mcu_interface/StartMcuCamTriggering.h"
#include "mcu_interface/PublishS10ToMcuOffset.h"

std::string IMU_TOPIC = "mcu_imu";
std::string IMU_TEMP_TOPIC_POSTFIX = "_temp";
std::string CAMERAS_TS_TOPIC = "mcu_cameras_ts";
std::string LIDAR_TS_TOPIC = "mcu_lidar_ts";
std::string S10_TS_TOPIC = "mcu_s10_ts";

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

const uint8_t OUTPUT_DATA_LENGTH_BYTES = 5;
uint8_t ALIGN_FRAMES_CMD = 34;
uint8_t START_TRIGGER_CMD = 56;
uint8_t STOP_TRIGGER_CMD = 57;

uint8_t output_buffer[OUTPUT_DATA_LENGTH_BYTES];
uint32_t CAMERAS_TIM_FRACT_NUMBER = 2560000; 
uint32_t alignment_subs = 0;
uint32_t alignment_subs_old = 0;
ros::Time last_cameras_ts = ros::Time(0);
float CAMERAS_FRAME_RATE = 30.0;
float SAMSUNG_CAMERA_FRAME_RATE = 30.0;
float SAMSUNG_CAMERA_FRAMING_PERIOD = 1.0 / SAMSUNG_CAMERA_FRAME_RATE;
double MANUAL_ADDITION = 0.005493;//-0.01243;//0.02064;//+0.0123; //sec
double phase_old = 0;

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

void publish_s10_ts(ros::Publisher pub, ros::Time mcu_ts, ros::Time s10_ts) {
    std::string frame_id = S10_TS_TOPIC;
    sensor_msgs::TimeReference msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = mcu_ts;
    msg.time_ref = s10_ts;
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

void send_to_mcu(serial::Serial *serial, uint8_t cmd, uint32_t data=0) {
    uint8_t buf[OUTPUT_DATA_LENGTH_BYTES];
    buf[0] = cmd;
    for(uint8_t i=1; i<OUTPUT_DATA_LENGTH_BYTES; i++) {
            buf[i] = (uint8_t)(data >> ((i-1) * 8));
    }
    serial->write(buf, OUTPUT_DATA_LENGTH_BYTES);
}
// Service server
bool align_phase(mcu_interface::AlignMcuCamPhase::Request  &req,
         mcu_interface::AlignMcuCamPhase::Response &res, serial::Serial *serial)
{
    double phase = req.a + MANUAL_ADDITION;
    ROS_WARN("phase %f", phase);

    double delta = phase - last_cameras_ts.toSec();
    ROS_WARN("last_cameras_ts.toSec() %f", last_cameras_ts.toSec());
    ROS_WARN("delta = phase - last_ts %f", delta);
    
    double delta_mod = std::fmod(delta, SAMSUNG_CAMERA_FRAMING_PERIOD);
    ROS_WARN("delta_mod %f", delta_mod);

    double delta_mod_pos = 0;
    if (delta_mod < 0) {
        delta_mod_pos = delta_mod + SAMSUNG_CAMERA_FRAMING_PERIOD;
    }
    ROS_WARN("delta_mod_pos %f", delta_mod_pos);

    double subs = delta_mod_pos * CAMERAS_TIM_FRACT_NUMBER * CAMERAS_FRAME_RATE; //-1
    ROS_WARN("subs %f", subs);
    
    alignment_subs = static_cast<uint32_t> (std::round(subs));
    //alignment_subs = 0x1A2B3C4D;//static_cast<uint32_t> (std::round(req.a));
    //ROS_WARN("alignment_subs %x\n", alignment_subs);
    ROS_WARN("alignment_subs %d\n", alignment_subs);
    send_to_mcu(serial, ALIGN_FRAMES_CMD, alignment_subs);
    //serial->write((uint8_t *)&alignment_subs, 4);
    
    // Stub
    res.response = "done";
    //ROS_WARN("sending back response: [%s]", res.response.c_str());

    // Writing debug info into file
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [90];
    strftime(buffer, 90, "/home/mrob/samsung-avatar-dataset-sync/out/mcu_interface/%m(%b)%d_%Y_%H%M%S.csv", now);
    std::ofstream my_file(buffer);
    //my_file.open (buffer);
    //std::string temp(buffer)
    //ROS_WARN("%s", temp.c_str());
    ROS_WARN(buffer);
    if(my_file.is_open()) {
        my_file << "phase," << phase << std::endl;
        my_file << "last_cameras_ts.toSec()," << last_cameras_ts.toSec() << std::endl;
        my_file << "delta = phase - last_ts," << delta << std::endl;
        my_file << "delta_mod," << delta_mod << std::endl;
        my_file << "delta_mod_pos," << delta_mod_pos << std::endl;
        my_file << "subs," << subs << std::endl;
        my_file << "alignment_subs," << alignment_subs << std::endl;
    }
    else {
        ROS_WARN("error");
    }
    my_file.close();

    return true;
}

bool start_triggering(mcu_interface::StartMcuCamTriggering::Request  &req,
         mcu_interface::StartMcuCamTriggering::Response &res, serial::Serial *serial)
{
    send_to_mcu(serial, START_TRIGGER_CMD);

    // Stub
    res.response = "done";
    ROS_WARN("start_triggering OK");
    return true;
}

bool publish_s10_to_mcu_offset(mcu_interface::PublishS10ToMcuOffset::Request  &req,
         mcu_interface::PublishS10ToMcuOffset::Response &res, ros::Publisher *pub)
{
    publish_s10_ts(*pub, ros::Time(last_cameras_ts), ros::Time(last_cameras_ts.toSec() + req.offset));
    // Stub
    res.response = "done";
    //ROS_WARN("sending back response: [%s]", res.response.c_str());
    return true;
}

int main(int argc, char **argv) {
    // Register signal and signal handler
    if (argc < 2) {
        std::cout << "Please, specify serial device. For example, \"/dev/ttyUSB0\"" << std::endl;
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
    ros::Publisher s10_ts_pub = nh.advertise<sensor_msgs::TimeReference>(S10_TS_TOPIC, 2, true);
    
	// Configure dynamic reconfigure
	dynamic_reconfigure::Server<mcu_interface::parametersConfig> server;
    dynamic_reconfigure::Server<mcu_interface::parametersConfig>::CallbackType f;
    f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
    server.setCallback(f);
	
	// open port, baudrate, timeout in milliseconds
    serial::Serial serial(PORT, BAUD, serial::Timeout::simpleTimeout(TIMOUT));
    
    // Service configure
    ros::ServiceServer phase_align_service = nh.advertiseService<mcu_interface::AlignMcuCamPhase::Request, mcu_interface::AlignMcuCamPhase::Response>(
        "align_mcu_cam_phase", boost::bind(align_phase, _1, _2, &serial));

    ros::ServiceServer camera_start_service = nh.advertiseService<mcu_interface::StartMcuCamTriggering::Request, mcu_interface::StartMcuCamTriggering::Response>(
        "start_mcu_cam_triggering", boost::bind(start_triggering, _1, _2, &serial));

    ros::ServiceServer publish_s10_to_mcu_offset_service = nh.advertiseService<mcu_interface::PublishS10ToMcuOffset::Request, mcu_interface::PublishS10ToMcuOffset::Response>(
        "publish_s10_to_mcu_offset", boost::bind(publish_s10_to_mcu_offset, _1, _2, &s10_ts_pub));


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

    //publish_s10_ts(s10_ts_pub, ros::Time(0.5), ros::Time(50));
    //ros::Time some1 = ros::Time::now();
    //uint8_t flag_some = 1;
    //serial.write((uint8_t *)&alignment_subs, 4);
    //send_to_mcu(&serial, START_TRIGGER_CMD);

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
                    //ROS_WARN("phase=phase*CAMERAS_TIM_FRACT_NUMBER %f", last_cameras_ts.toSec());
					break;
				}
				case 'l': {
                    publish_lidar_ts(lidar_ts_pub, ts);
                    //ROS_WARN("%s", str.c_str());
                    //std::cout << ts << std::endl;
                    //publish_s10_ts(s10_ts_pub, ros::Time(0.5), ros::Time(50));
                    break;
                }
                case 't': {
                    ROS_WARN("%s", str.c_str());
                    break;
                }
			}
			ros::spinOnce();// this checks for callbacks (dynamic reconfigure)
	    }
        //if (flag_some == 1 && (ros::Time::now() - some1).toSec() > 5) {
            //send_to_mcu(&serial, START_TRIGGER_CMD);
            //serial.write((uint8_t *)&alignment_subs, 5);
            //flag_some = 0;
            //ROS_WARN("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        //}
	    usleep(100);
	}
    send_to_mcu(&serial, STOP_TRIGGER_CMD);
}
