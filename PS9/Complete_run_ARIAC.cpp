//Final assignment for ARIAC
//Akhil Kandhari, axk751@case.edu
//Created 12/9/2018

//Following code has been developed using unload_box_v4.cpp, box_inspector.cpp

/*
This node is used for making corrections at Quality station 1 where a defected part is picked and removed followed by replacing the part,
the box then advances to Quality station 2 where manually we move around the gears and then by making adjustments the robot will precisely place 
the parts in the box.

->Update box_inspector since function for box inspection at Q2 does not exist.
->Use BinInventory library to aquire new parts from inventory and to do part location adjustments
->Invoke additional robot behavior functions to fetch part from inventory and place it in the box.
*/

#include <ros/ros.h>
#include <std_srvs/Trigger.h> //Include service type for starting competition
#include <robot_behavior_interface/RobotBehaviorInterface.h> //use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <inventory_msgs/Part.h> //we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include <box_inspector/box_inspector2.h> //a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include <conveyor_as/ConveyorInterface.h> //conveyor interface communicates with the conveyor action server
#include <bin_inventory/bin_inventory.h>
//Library for transforms
#include <xform_utils/xform_utils.h>

XformUtils xformUtils_;

const double COMPETITION_TIMEOUT = 500.0; //Want to ship before time runs out

vector<osrf_gear::Model> desired_models_wrt_box,desired_models_wrt_world;
int num_desired_parts;

osrf_gear::Order g_order;
bool g_got_order = false;

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

// Listening for the Orders from ARIAC

void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

    g_order = *msg;
    g_got_order = true;
    ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "" : "s");
    ROS_INFO_STREAM(g_order);
    //Populate desired_models_wrt_box first:

    /*
    osrf_gear/model:
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

    osrf_gear/order:
    string order_id
    osrf_gear/Shipment[] shipments
        string shipment_type
        osrf_gear/Product[] products
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

*/

    

    num_desired_parts = g_order.shipments[0].products.size();
    ROS_INFO("Number of parts required for this shipment: %d",num_desired_parts);

    desired_models_wrt_box.resize(num_desired_parts);
    desired_models_wrt_world.resize(num_desired_parts);

    //desired_models_wrt_box[0].type=g_order.shipments[0].products[0].type;
    //desired_models_wrt_box[0].pose=g_order.shipments[0].products[0].pose;

    //ROS_INFO_STREAM(desired_models_wrt_box);
    //Populate position wrt box of these parts:

    for(int i=0;i<num_desired_parts;i++){
        desired_models_wrt_box[i].type=g_order.shipments[0].products[i].type;
        desired_models_wrt_world[i].type=desired_models_wrt_box[i].type;
        desired_models_wrt_box[i].pose=g_order.shipments[0].products[i].pose;
    }

    ROS_INFO("Parts in box should be: ");
    ROS_INFO_STREAM(desired_models_wrt_box[0]);
    ROS_INFO_STREAM(desired_models_wrt_box[1]);
    ROS_INFO_STREAM(desired_models_wrt_box[2]);

}

//Function to compute position of parts within the box in the world frame
//This should be moved to box_inspector, however do that later if needed.
void model_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world,vector<osrf_gear::Model> &model_wrt_box,int num_comp)
{
    Eigen::Affine3d box_wrt_world;
    vector<Eigen::Affine3d> parts_wrt_box, parts_wrt_world;

    parts_wrt_world.resize(num_comp);
    parts_wrt_box.resize(num_comp);
    
    box_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);

    for(int i=0;i<num_comp;i++){
        parts_wrt_box[i]=xformUtils_.transformPoseToEigenAffine3d(model_wrt_box[i].pose);
        parts_wrt_world[i]=box_wrt_world*parts_wrt_box[i];
        desired_models_wrt_world[i].pose=xformUtils_.transformEigenAffine3dToPose(parts_wrt_world[i]);
        ROS_INFO("Parts in world frame: ");
        ROS_INFO_STREAM(desired_models_wrt_world[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Run_Ariac_Simulation");
    ros::NodeHandle nh;

    int ans;

    unsigned char box_location_code = inventory_msgs::Part::QUALITY_SENSOR_1;

    //First step is to start the competition based on PS5:

    ros::ServiceClient startup_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger StartupSrv;

    startup_client.call(StartupSrv);

    while(StartupSrv.response.success == false){
        ROS_WARN("Competition start not successful");
        startup_client.call(StartupSrv);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Competition start successful!");

    //Start interface with robot
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    //Start interface with conveyor belt
    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    //Start interface with box inspector
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector2 boxInspector(&nh);

    //Start interface with bin inspector
    ROS_INFO("instantiating a binInventory object");
    BinInventory binInventory(&nh);
    inventory_msgs::Inventory current_inventory;

    //Start interface with drone
    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl droneControl;

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part, desired_part;

    //camera sees box, coordinates are converted to world coords
    geometry_msgs::PoseStamped box_pose_wrt_world; 

    bool status;    
    int nparts;

    ros::Subscriber sub = nh.subscribe("ariac/orders", 5, orderCallback);
    ROS_INFO("waiting for order...");
    while (!g_got_order) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (!g_got_order) ROS_INFO("waiting");

    }    

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
    conveyorInterface.move_new_box_to_Q1(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //Check to see if box made it to Q1 
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world,CAM1)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("no box seen. something is wrong! I quit!!");
        exit(1);
    }

    model_pose_wrt_world(box_pose_wrt_world,desired_models_wrt_box,num_desired_parts);
    //At Q1 check for bad components using camera 1    
    boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM1);


    if (boxInspector.get_bad_part_Q(current_part,CAM1)) {
        ROS_INFO("Found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);
        
        //use the robot action server to acquire and dispose of the specified part in the box:
        status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);

        ROS_INFO("Bad part removed");
        //ros::Duration(5.0).sleep();
    }

    bool go_on = false;
    int n_missing_part = part_indices_missing[0];

    cout<<n_missing_part<<endl;
    ros::Duration(5.0).sleep();
    //model_to_part(desired_models_wrt_world[n_missing_part], current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
    std::string part_name(desired_models_wrt_world[n_missing_part].type);

    ROS_INFO_STREAM("looking for part " << part_name << endl);
    int partnum_in_inventory;
    bool part_in_inventory = true;
    inventory_msgs::Part pick_part, place_part;

    binInventory.update();
    binInventory.get_inventory(current_inventory);
    part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
    if (!part_in_inventory)
    {
        ROS_WARN("could not find desired part in inventory; giving up on process_part()");
        return false; //nothing more can be done
    }
    ROS_INFO_STREAM("found part: " << pick_part << endl);
    //specify place part: The next line is not working since none of the above has been computed:
    model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_1);

    go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
    if (!go_on)
    {
        ROS_WARN("could not compute key pickup and place poses for this part source and destination");
    }

    ROS_INFO("attempting pick...");
    ROS_INFO("attempting to pick part");
    if (!robotBehaviorInterface.pick_part_from_bin(pick_part))
    {
        ROS_INFO("pick failed");
        go_on = false;
        return false;
    }

    ROS_INFO("moving to approach pose");
    if (!robotBehaviorInterface.move_part_to_approach_pose(place_part))
    {
        ROS_WARN("could not move to approach pose");
        go_on = false;
        robotBehaviorInterface.discard_grasped_part(place_part);
    }
    //place  part:
    ROS_INFO("attempting to place part");

    if (!robotBehaviorInterface.place_part_in_box_no_release(place_part))
    {
        ROS_INFO("placement failed");
        go_on = false;
        return false;
    }

    status = robotBehaviorInterface.release_and_retract();

    ROS_INFO("Part has been replaced with new component");

    //ros::Duration(10.0).sleep();
    /////////////////////////////////////////////////////////////////////////////////////////////    


    //Move box from Q1 to Q2
    conveyorInterface.move_box_Q1_to_Q2(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q2...");
        }
    }

    //ros::Duration(5.0).sleep();

    geometry_msgs::PoseStamped box_pose_wrt_world_2; 
    //Check to see if box made it to Q2 
    if (boxInspector.get_box_pose_wrt_world2(box_pose_wrt_world_2)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world_2 << endl);
    }
    else {
        ROS_WARN("no box seen. something is wrong! I quit!!");
        exit(1);
    }

    desired_models_wrt_box.clear();
    model_pose_wrt_world(box_pose_wrt_world_2,desired_models_wrt_box,num_desired_parts);

    //At station 2 move parts such that pose is different than exact
    ROS_INFO("PAUSING....... DISPLACE PARTS HERE PLEASE");
    ros::Duration(5.0).sleep();

    boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM2);
    nparts = orphan_models_wrt_world.size();
    
    int nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
    ROS_INFO("found %d misplaced parts", nparts_misplaced);

    ros::Duration(5.0).sleep();

    box_location_code = inventory_msgs::Part::QUALITY_SENSOR_2;

    for(int i =0;i<nparts_misplaced;i++){
        model_to_part(misplaced_models_actual_coords_wrt_world[i], current_part, box_location_code);
        //adjust_part_location_no_release(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
        int index_des_part = part_indices_misplaced[i];
        model_to_part(desired_models_wrt_world[index_des_part], desired_part, box_location_code);
        ROS_INFO("move part from: ");
        ROS_INFO_STREAM(current_part);
        ROS_INFO("move part to: ");
        ROS_INFO_STREAM(desired_part);
        //use the robot action server to grasp part in the box:
        status = robotBehaviorInterface.pick_part_from_box(current_part);

        //following fnc works ONLY if part is already grasped:
        status = robotBehaviorInterface.adjust_part_location_no_release(current_part, desired_part);
        status = robotBehaviorInterface.release_and_retract();
    }


    ROS_INFO("advancing box to loading dock for shipment");
    conveyorInterface.move_box_Q2_to_drone_depot(); //member function of conveyor interface to move a box to shipping dock

    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to loading dock...");
        }
    }
    ROS_INFO("calling drone");
    
    droneControl.request.shipment_type = g_order.shipments[0].shipment_type;
    ROS_INFO_STREAM("shipment name: " << g_order.shipments[0].shipment_type << endl);

    droneControl.response.success = false;
    while (!droneControl.response.success) {
        drone_client.call(droneControl);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("all done; goodbye, world");
    return 0;


}