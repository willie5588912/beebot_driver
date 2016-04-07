#include <SerialStream.h>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

using namespace LibSerial ;    

float g_linear_x= 0;
float g_angular_z= 0;
const float G_MAX_VEL= 1000;

#if 1
class SerialHandle
{
	public:
		void init(const char* const p_port_name);
		void set_baudRate(int p_baud_rate);
		void set_dataBits(int p_data_bits);
		void set_stopBit(int p_stop_bit);
		void set_parity(bool p_parity);
		void set_hardwareFlowControl(bool p_control);
		void writeData(std::string p_data);
	private:
    	SerialStream serial_port;
		int _baud_rate;
		int _data_bits;
		int _stop_bit;
		bool _flow_control;

} g_my_port;
#endif

//open the serial port
void SerialHandle::init(const char* const p_port_name)
{
	const char* const SERIAL_PORT_DEVICE = p_port_name;
    serial_port.Open( SERIAL_PORT_DEVICE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not open serial port " 
                  << SERIAL_PORT_DEVICE 
                  << std::endl ;
        exit(1) ;
    }
}

//set baud rate
void SerialHandle::set_baudRate(int p_baud_rate)
{
	_baud_rate= p_baud_rate;
	if(_baud_rate == 9600)
	{
		serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
		if ( ! serial_port.good() ) 
		{
			std::cerr << "Error: Could not set the baud rate." << std::endl ;
			exit(1) ;
		}
	}
	else
	{
		ROS_ERROR("TO DO: modify code to fit other baud rate");
		exit(1);
	}
}

//set the num of data bits
void SerialHandle::set_dataBits(int p_data_bits)
{
	_data_bits= p_data_bits;
	if(_data_bits == 8)
	{
		serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
		if ( ! serial_port.good() ) 
		{
			std::cerr << "Error: Could not set the character size." << std::endl ;
			exit(1) ;
		}
	}
	else
	{
		ROS_ERROR("TO DO: modify code to fit other data bits.");
		exit(1);
	}
}

//set stop bit
void SerialHandle::set_stopBit(int p_stop_bit)
{
	_stop_bit= p_stop_bit;
	serial_port.SetNumOfStopBits(_stop_bit) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }

}

//set parity
void SerialHandle::set_parity(bool p_parity)
{
	//disable parity
	if(!p_parity)
	{
		serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
		if ( ! serial_port.good() ) 
		{
			std::cerr << "Error: Could not disable the parity." << std::endl ;
			exit(1) ;
		}
	}
	else
	{
		ROS_ERROR("TODO: enble parity");
		exit(1);
	}
}

//set hardware flow control
void SerialHandle::set_hardwareFlowControl(bool p_control)
{
	//enable hardware flow control
	if(p_control)
	{
		serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
		if ( ! serial_port.good() ) 
		{
			std::cerr << "Error: Could not use hardware flow control."
					  << std::endl ;
			exit(1) ;
		}
	}
	else
	{
		ROS_ERROR("TODO: disable flow contorl");
		exit(1);
	}
}

//write data
void SerialHandle::writeData(std::string p_data)
{
//	std::cerr << "Dumping file to serial port." << std::endl ;
	ROS_INFO("%s\n", p_data.c_str());
	serial_port << p_data << std::endl ;
//    std::cerr << std::endl ;
//    std::cerr << "Done." << std::endl ;
}

void control_judge()
{
	std::stringstream ss1, ss2;

	//move forward or backward
	if(g_angular_z == 0)
	{
		ss1 << "1v" << -g_linear_x << "\r";
		ss2 << "2v" << g_linear_x << "\r";
	}
	//turn left
	else if(g_linear_x == 0 && g_angular_z >= 0)
	{
		ss1 << "1v" << g_angular_z << "\r";
		ss2 << "2v" << g_angular_z << "\r";
	}
	//turn right
	else if(g_linear_x ==0 && g_angular_z < 0)
	{
		ss1 << "1v" << g_angular_z << "\r";
		ss2 << "2v" << g_angular_z << "\r";
	}

	g_my_port.writeData(ss1.str());
	g_my_port.writeData(ss2.str());
}


void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
	g_linear_x= (twist_msg->linear.x)*G_MAX_VEL;
	g_angular_z= (twist_msg->angular.z)*G_MAX_VEL;
}

int main(int argc, char **argv)
{    
	ros::init(argc, argv, "beebot_driver");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::Rate loop_rate(10);

	std::string port;
	if(!private_nh.getParam("port", port))
		port = "/dev/ttyUSB0";

	ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1000, twistCallback);

	g_my_port.init(port.c_str());
	g_my_port.set_baudRate(9600);
	g_my_port.set_dataBits(8);
	g_my_port.set_stopBit(1);
	g_my_port.set_parity(false);
	g_my_port.set_hardwareFlowControl(true);

	//initial Faulhaber motor
	g_my_port.writeData("en\r");

	while (ros::ok())
	{
		ros::spinOnce();
		control_judge();
	  	loop_rate.sleep();
	}

    return 0;
}
