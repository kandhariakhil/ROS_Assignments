//unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>

//Type of message being sent via LogicalCamera at QS1
//rosmsg show osrf_gear/LogicalCameraImage for more information
#include<osrf_gear/LogicalCameraImage.h>

#include<geometry_msgs/Pose.h>

//Library for transforms
#include <xform_utils/xform_utils.h>

XformUtils xformUtils_;

//Message info: osrf_gear/Model[] models

/*
osrf_gear/Model[] models  
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w 
*/

geometry_msgs::Pose logical_camera_pose, defective_part_pose, part_pose_wrt_world,final_defective_part;

const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}



//Callback added here:
bool done_log=false;
bool done_box= false;

void quality_control_A1_callback(const osrf_gear::LogicalCameraImage& logical_image)
{
    ROS_INFO("Received information");
    //cout<<logical_image;
    
    if(logical_image.models.size()>0){
    defective_part_pose = logical_image.models[0].pose;
    logical_camera_pose = logical_image.pose;

    ROS_INFO("Defective part pose based on camera position: ");
    //cout<<defective_part_pose<<endl;

    ROS_INFO("Camera position in world: ");
    //cout<<logical_camera_pose<<endl;

    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;

    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(logical_camera_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(defective_part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    part_pose_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);

    ROS_INFO("Part pose wrt world");
    cout<<part_pose_wrt_world<<endl;
    }

    done_log = true;
}

void box_camera_A1_callback(const osrf_gear::LogicalCameraImage& box_cam_image)
{
    ROS_INFO("Comparing with box camera");

    geometry_msgs::Pose model_1,model_2,model_3,box_cam_pose,model_1_world,model_2_world,model_3_world;

    model_1 = box_cam_image.models[1].pose;
    model_2 = box_cam_image.models[2].pose;
    model_3 = box_cam_image.models[3].pose;

    box_cam_pose = box_cam_image.pose;

    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;

    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(model_1);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    model_1_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);

    cout<<model_1_world<<endl;

    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(model_2);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    model_2_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);

    cout<<model_2_world<<endl;

    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(model_3);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    model_3_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);

    cout<<model_3_world<<endl;

    /*
    if(model_1_world == part_pose_wrt_world)
        ROS_INFO("Defective part: Piston rod");
        cout<<model_1_world<<endl;
        final_defective_part = model_1_world;
    else if(model_2_world == part_pose_wrt_world)
        ROS_INFO("Defective part: gear part");
        cout<<model_2_world<<endl;
        final_defective_part = model_2_world;
    else if(model_3_world == part_pose_wrt_world)
        ROS_INFO("Defective part: gear part");
        cout<<model_2_world<<endl;
        final_defective_part = model_3_world;
    else
        ROS_INFO("Defective part lost in space")
    */

   done_box = true;
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;
    
 
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);
   
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part;

    geometry_msgs::PoseStamped box_pose_wrt_world;  //camera sees box, coordinates are converted to world coords
    
    bool status;    
    int nparts;

    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
        vector<osrf_gear::Model> desired_models_wrt_world;
        vector<osrf_gear::Model> satisfied_models_wrt_world;
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
        vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
        vector<osrf_gear::Model> missing_models_wrt_world;
        vector<osrf_gear::Model> orphan_models_wrt_world;
        vector<int> part_indices_missing;
        vector<int> part_indices_misplaced;
        vector<int> part_indices_precisely_placed;

    
    //use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1();  //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //update box pose,  if possible              
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("no box seen. something is wrong! I quit!!");
        exit(1);
    }

    //Here we modify the code such that we can subscribe to the quality control sensor and see what the coordinates of the bad part are and print those coordinates
    //for comparing it with box_camera_1 and once we have the part we send the pose of the part in world frmae to the robot to pick up that.

    //Subscriber added here:

    ros::Rate loop_rate(100);

    ros::Subscriber quality_control_subscriber = nh.subscribe("/ariac/quality_control_sensor_1",1,quality_control_A1_callback);
        
    while(done_log==false && ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Logical Camera at Quality Station 1 has reported some defective components");

/*
    ros::Subscriber box_control_subscriber = nh.subscribe("/ariac/box_camera_1",1,box_camera_A1_callback);

    while(!done_box && ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("All comparisons done, removing parts now");

    ros::Duration(5.0).sleep();
*/
    if (boxInspector.get_bad_part_Q1(current_part)) {
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);

        ros::Duration(10.0).sleep();
        
        status = robotBehaviorInterface.pick_part_from_box(current_part);

        ROS_INFO("PART WAS REMOVED WAITING FOR REINSPECTION");
        ros::Duration(10.0).sleep();

    //use the robot action server to acquire and dispose of the specified part in the box:
    }    

    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

    //use the robot action server to acquire and dispose of the specified part in the box:
    status = robotBehaviorInterface.pick_part_from_box(current_part);
    ros::Duration(10.0).sleep();

    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

    //use the robot action server to acquire and dispose of the specified part in the box:
    status = robotBehaviorInterface.pick_part_from_box(current_part);
    ros::Duration(10.0).sleep();

    //SHOULD REPEAT FOR ALL THE PARTS IN THE BOX
    //ALSO, WATCH OUT FOR NO PARTS IN THE BOX--ABOVE WILL CRASH

            return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}
