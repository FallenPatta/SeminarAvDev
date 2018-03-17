#include <thread>
#include <random>
#include <condition_variable>

#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//ROS Subscriber Objekt anlegen
ros::Subscriber joystick_subscriber;
bool joystick_offline = true;

//ROS Publisher Objekt anlegen
ros::Publisher command_publisher;
std::thread command_generation_thread;
double * current_command;
bool sending_permitted = true;

//Mutex und condition_variable zum warten anlegen
std::mutex awaiting_command_mutex;
std::condition_variable command_notification_condition;
bool command_ready = false;
bool allow_next_command = false;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& input){
	sending_permitted = false;
	joystick_offline = false;
}

void commandThread(){
	//Update Thread läuft mit 2 Hz
	ros::Rate sleep(2.0);
	while(ros::ok())
	{
		//Warten, wenn die Main Funktion ein neues Kommando erzeugen will
		if(!command_ready)
		{
			allow_next_command = true;
			command_notification_condition.notify_one();
			std::unique_lock<std::mutex> lock(awaiting_command_mutex);
			command_notification_condition.wait(lock, []{return command_ready;});
		}
		
		//Twist Message erzeugen
		geometry_msgs::Twist driveMessage;
		driveMessage.linear.x = current_command[1]/10.0;
		driveMessage.angular.z = current_command[1]/current_command[0];
		
		//Wenn publishen erlaubt ist
		if(sending_permitted)
		{
			//Publishen
			command_publisher.publish(driveMessage);
			ROS_INFO("Publisher: %f, %f", driveMessage.linear.x, driveMessage.angular.z);
		}
		
		//Für ~0.5 Sekunden warten
		sleep.sleep();
	}
}

int main(int argc, char** argv)
{
	//Knoten starten
	ROS_INFO("Starting the Node");
	ros::init(argc, argv, "com_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	
	//Lokale Variablen für die Parameter anlegen
	double radius = 400;
	double min_velocity = 0.0;
	double max_velocity = 0.0;
	std::string output_topic = "/cmd_vel";
	
	//Parameterserver auslesen
	ros::param::get("/com_node/radius", radius);
	ros::param::get("/com_node/min_velocity", min_velocity);
	ros::param::get("/com_node/max_velocity", max_velocity);
	ros::param::get("/com_node/output_topic", output_topic);
	
	//Gelesene Parameter ausgeben
	ROS_INFO("radius: %f", radius);
	ROS_INFO("min_velocity: %f", min_velocity);
	ROS_INFO("max_velocity: %f", max_velocity);
	ROS_INFO("output_topic: %s", output_topic.c_str());
	
	//Publisher starten
	command_publisher = n.advertise<geometry_msgs::Twist>(output_topic, 5, true);
	
	//Subscriber starten
	joystick_subscriber = n.subscribe<sensor_msgs::Joy>("/joy", 10, joystickCallback);
	
	//Kommandovariable initialisieren
	current_command = new double[2]();
	current_command[0] = radius;
	current_command[1] = min_velocity;
	
	//Zufallsgenerator initialisieren
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform_velocity(min_velocity, max_velocity);
    
    //ROS Spinner starten
    ros::AsyncSpinner spinner(4);
	spinner.start();
	
	//Kommandothread starten
	command_generation_thread = std::thread(commandThread);
	
	//Exit blockieren solange ROS aktiv ist
	ros::Rate sleep(0.2);
	while(ros::ok())
	{
		//Publisher Thread stoppen
		command_ready = false;
		{
			std::unique_lock<std::mutex> lock(awaiting_command_mutex);
			command_notification_condition.wait(lock, []{return allow_next_command;});
		}
		
		//Neue Kommandos erzeugen
		current_command[0] = radius;
		current_command[1] = uniform_velocity(gen);
		command_ready = true;
		allow_next_command = false;
		
		//Publisher Thread wieder starten
		command_notification_condition.notify_one();
		
		//Wartezeit anpassen und publishen verbieten, wenn Joystick aktiv war
		bool waiting20sec = false;
		if(!joystick_offline)
		{
			joystick_offline = true; // Vor dem Wartevorgang zurücksetzen
			sleep = ros::Rate(0.05);
			ROS_INFO("joystick is not offline - delaying actions and checking again in 20 seconds");
			waiting20sec = true;
		}
		
		//Bis zur nächsten Iteration warten
		sleep.sleep();
		
		//Wenn Joystick mindestens 20 Sekunden offline war, die Wartezeit wieder zurücksetzen und publishen erlauben
		if(joystick_offline && waiting20sec)
		{
			sleep = ros::Rate(0.2);
			sending_permitted = true;
			ROS_INFO("joystick is still offline after waiting 20 seconds - publisher will start again");
		}
	}
	
	return 1;
}
