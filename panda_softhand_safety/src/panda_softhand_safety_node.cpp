#include "panda_softhand_safety/collision_evader.h"
#include "panda_softhand_safety/SafetyInfo.h"

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "panda_softhand_safety_node");
    ros::NodeHandle nh;

    // Safety info message
    panda_softhand_safety::SafetyInfo safety_info_msg;

    // Collision evader object
    panda_softhand_safety::CollisionEvader collision_evader(nh);

    // Starting to spin
    while (ros::ok()) {
        
        ros::spinOnce();

        // Clearing the message

        // Checking for collisions
        if (!collision_evader.EvadeCollision()){
            
        }
    }

    ros::shutdown();
    return 0;
}
