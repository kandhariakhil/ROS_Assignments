#include<ros/ros.h>
#include<std_msgs/Float64.h>


//Written for practice, please follow Newman's part 1 my_sleepy_minimal_publisher for more information.
int main(int argc, char **argv){
	ros::init(argc,argv,"minimal_publisher2");
	ros::NodeHandle n;
	ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("topic1",1);

	std_msgs::Float64 input_float;

	ros::Rate naptime(1.0);
	
	input_float.data = 0.0;

	while(ros::ok()){
	
		input_float.data = input_float.data+0.001;
		my_publisher_object.publish(input_float);
		naptime.sleep();
	}
}
