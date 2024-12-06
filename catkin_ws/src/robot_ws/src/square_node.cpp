#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <cstdlib>

enum State{
    FORWARD,
    TURN,
    STOP,
    IDLE
};

State current_state = IDLE;
int state_counter = 0;

ros::Time state_start_time;

void move_forward(ros::Publisher& distance_pub, int distance) {
    if(current_state == FORWARD){
        std_msgs::Int32 distance_msg;
        distance_msg.data = distance; 
        distance_pub.publish(distance_msg);
        ROS_INFO("Moving forward: %d meters", distance);
        ros::Duration(6).sleep(); 
        current_state = TURN;
        state_start_time = ros::Time::now();
    }
}

void rotate_90(ros::Publisher& angle_pub, int angle) {
    if (current_state == TURN){
        std_msgs::Int32 angle_msg;
        angle_msg.data = angle;  
        angle_pub.publish(angle_msg);
        ROS_INFO("Rotating 90 degrees");
        ros::Duration(4).sleep(); 
        current_state = FORWARD;
        state_counter++;
        state_start_time = ros::Time::now();
    }
}

void checkStateTimeout() {
    // Cambia el estado a STOP si pasa el tiempo de cada acción sin completarse
    if ((ros::Time::now() - state_start_time).toSec() > 5.0) {
        current_state = STOP;
        ROS_WARN("State timed out, stopping motors");
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "square_node");
    ros::NodeHandle nh;

    // Publicadores de movimiento y rotación
    ros::Publisher distance_pub = nh.advertise<std_msgs::Int32>("motor_distanceCentimeter", 1000);
    ros::Publisher angle_pub = nh.advertise<std_msgs::Int32>("motor_AngleAxis", 1000);
    ros::Rate loop_rate(10);

    int distance = 60;  // Distancia de avance en metros
    int angle = 90;    // Ángulo de rotación en grados
    ros::Duration(1).sleep(); 
    current_state = FORWARD;  // Inicia en FORWARD
    state_start_time = ros::Time::now();

    while (ros::ok()) {
        switch (current_state) {
            case FORWARD:
                move_forward(distance_pub, distance);
                break;
            case TURN:
                rotate_90(angle_pub, angle);
                break;
            case STOP:
                ROS_WARN("Motors stopped due to timeout");
                break;
            default:
                break;
        }

        // Verifica el tiempo de espera
        checkStateTimeout();

        // Verifica si completamos el cuadrado
        if (state_counter >= 4 && current_state == FORWARD) {
            current_state = STOP;
            ROS_INFO("Square completed");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

    
}






