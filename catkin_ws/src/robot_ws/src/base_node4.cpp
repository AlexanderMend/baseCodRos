#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include "roboclaw.hpp"  // Incluye tu librería Roboclaw
#include <std_msgs/Int32MultiArray.h>
#include <cstdlib>

// Inicializa Roboclaw con el puerto adecuado y baudrate
Roboclaw roboclaw("/dev/ttyACM0", 115200);

int32_t motor_speed1 = 12000;
int32_t motor_speed2 = 8000;

void commandCallbackAngleAxis(const std_msgs::Int32::ConstPtr& msg){
    float Kp2 = 2.12612, Ki2 = 0.30906, Kd2 = 0.0;
    const uint32_t qpps2 = 12562;  // Pulsos por segundo

    roboclaw.SetM2VelocityPID(0x80, Kp2, Ki2, Kd2, qpps2);
    int32_t AngleAxis = msg -> data;
    AngleAxis = AngleAxis*182;

    roboclaw.SpeedDistanceM2(0x80,motor_speed2,AngleAxis,1);

}

void commandCallbackDistanceCentimeter(const std_msgs::Int32::ConstPtr& msg) {
    float Kp1 = 1.31034, Ki1 = 0.12314, Kd1 = 0.0;  
    uint32_t qpps1 = 56500;  // Pulsos por segundo

    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);
    int32_t distance = msg->data; 
    distance = distance*585;

    roboclaw.SpeedDistanceM1(0x80, motor_speed1, distance, 1);
}



void commandCallbackSpeed1(const std_msgs::Int32::ConstPtr& msg) {
    motor_speed1 = msg->data;  // Actualiza la velocidad con el valor recibido en el mensaje
    ROS_INFO("Updated Motor Speed M1 to: %d", motor_speed1);
}

void commandCallbackDistance1(const std_msgs::Int32::ConstPtr& msg) {
    float Kp1 = 1.31034, Ki1 = 0.12314, Kd1 = 0.0;  
    uint32_t qpps1 = 56500;  // Pulsos por segundo

    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);
    int32_t distance1 = msg->data; 
    roboclaw.SpeedDistanceM1(0x80, motor_speed1, distance1, 1);
}

void commandCallbackSpeed2(const std_msgs::Int8::ConstPtr& msg) {
    motor_speed2 = msg->data;  // Actualiza la velocidad con el valor recibido en el mensaje
    ROS_INFO("Updated Motor Speed M2 to: %d", motor_speed2);
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    // Subscriptores para velocidad y distancia para cada motor
    ros::Subscriber subSpeed1 = n.subscribe("motor_speed1", 1000, commandCallbackSpeed1);
    ros::Subscriber subDistance1 = n.subscribe("motor_distance1", 1000, commandCallbackDistance1);
    ros::Subscriber subSpeed2 = n.subscribe("motor_speed2", 1000, commandCallbackSpeed2);
    ros::Subscriber subDistanceMeter = n.subscribe("motor_distanceCentimeter",1000,commandCallbackDistanceCentimeter);
    ros::Subscriber subAngleAxis = n.subscribe("motor_AngleAxis",1000,commandCallbackAngleAxis);

    // Publicadores para monitorear la velocidad y distancia de cada motor
    ros::Publisher speed_pub1 = n.advertise<std_msgs::Int32>("motor1_speed_feedback", 1000);
    ros::Publisher distance_pub1 = n.advertise<std_msgs::Int32>("motor1_distancePulses_feedback", 1000);
    ros::Publisher speed_pub2 = n.advertise<std_msgs::Int32>("motor2_speed_feedback", 1000);
    ros::Publisher distance_pub2 = n.advertise<std_msgs::Int32>("motor2_pulses_feedback", 1000);

    ros::Rate loop_rate(10);  // Frecuencia de 10Hz

    while (ros::ok()) {
        uint8_t status1, status2;
        bool validspeed1, validspeed2;
        bool validdistance1, validdistance2;

        int32_t motor_speed1 = roboclaw.ReadSpeedM1(0x80, &status1, &validspeed1);
        int32_t motor_distance1 = roboclaw.ReadEncM1(0x80, &status1, &validdistance1);
        int32_t motor_speed2 = roboclaw.ReadSpeedM2(0x80, &status2, &validspeed2);
        int32_t motor_distance2 = roboclaw.ReadEncM2(0x80, &status2, &validdistance2);

        if (validspeed2) {
            std_msgs::Int32 speed_msg2;
            speed_msg2.data = motor_speed2;
            speed_pub2.publish(speed_msg2);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M2");
        }

        if (validspeed1) {
            std_msgs::Int32 speed_msg1;
            speed_msg1.data = motor_speed1;
            speed_pub1.publish(speed_msg1);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M1");
        }

        if (validdistance2) {
            std_msgs::Int32 distance_msg2;
            distance_msg2.data = motor_distance2;
            speed_pub2.publish(distance_msg2);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M2");
        }

        if (validdistance1) {
            std_msgs::Int32 distance_msg1;
            distance_msg1.data = motor_distance1;
            speed_pub1.publish(distance_msg1);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M1");
        }

        // Publica la velocidad y distancia de cada motor de forma independiente
        std_msgs::Int32 speed_msg1;
        speed_msg1.data = motor_speed1;
        speed_pub1.publish(speed_msg1);

        std_msgs::Int32 distance_msg1;
        distance_msg1.data = motor_distance1;
        distance_pub1.publish(distance_msg1);

        std_msgs::Int32 speed_msg2;
        speed_msg2.data = motor_speed2;
        speed_pub2.publish(speed_msg2);

        std_msgs::Int32 distance_msg2;
        distance_msg2.data = motor_distance2;
        distance_pub2.publish(distance_msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
