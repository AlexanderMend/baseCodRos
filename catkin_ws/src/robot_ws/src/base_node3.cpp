#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include "roboclaw.hpp"  // Incluye tu librería Roboclaw
#include <std_msgs/Int32MultiArray.h>
#include <cstdlib>

// Inicializa Roboclaw con el puerto adecuado y baudrate
Roboclaw roboclaw("/dev/ttyACM0", 115200);
// Variables globales para almacenar los datos de distancia y velocidad recibidos
int32_t distance1 = 0, speed1 = 0;
int32_t distance2 = 0, speed2 = 0;
void commandCallback1(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    
    uint8_t address = 0x80;     // Dirección del Roboclaw

    float Kp1 = 1.31034, Ki1 = 0.12314, Kd1 = 0.0;  
    uint32_t qpps1 = 56500;  // Pulsos por segundo

    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);

    if (msg->data.size() >= 2) {  // Asegura que hay al menos dos elementos
        distance1 = msg->data[0];
        speed1 = msg->data[1];
        ROS_INFO("Motor M1 received distance: %d, speed: %d", distance1, speed1);

        roboclaw.SpeedDistanceM1(address, speed1, distance1, 1); // Modo de aceleración fija
    } else {
        ROS_ERROR("Received array for Motor 1 does not have enough elements");
    }

    // Establece la velocidad del motor 1 usando la función SpeedM1
    /*
    if (roboclaw.SpeedM1(address, speed)) {
        ROS_INFO("Motor M1 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M1");
    }

    // Establece la velocidad del motor 2 usando la función SpeedM2
    if (roboclaw.SpeedM2(address, speed-speed)) {
        ROS_INFO("Motor M2 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M2");
    }
    */
}  

void commandCallback2(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        uint8_t address = 0x80;     // Dirección del Roboclaw

    float Kp2 = 2.12612, Ki2 = 0.30906, Kd2 = 0.0;
    const uint32_t qpps2 = 12562;  // Pulsos por segundo

    roboclaw.SetM2VelocityPID(0x80, Kp2, Ki2, Kd2, qpps2);

    if (msg->data.size() >= 2) {  // Asegura que hay al menos dos elementos
        distance2 = msg->data[0];
        speed2 = msg->data[1];
        ROS_INFO("Motor M2 received distance: %d, speed: %d", distance2, speed2);

        roboclaw.SpeedDistanceM2(address, speed2, distance2, 1); // Modo de aceleración fija
    } else {
        ROS_ERROR("Received array for Motor 2 does not have enough elements");
    }
          // M1 retrocede
    } 

    // Establece la velocidad del motor 1 usando la función SpeedM1
    /*
    if (roboclaw.SpeedM1(address, speed)) {
        ROS_INFO("Motor M1 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M1");
    }

    // Establece la velocidad del motor 2 usando la función SpeedM2
    if (roboclaw.SpeedM2(address, speed-speed)) {
        ROS_INFO("Motor M2 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M2");
    }
    */


int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    // No se requiere "connect()" ya que al crear la instancia de Roboclaw, el puerto se abre.

    ros::Subscriber sub1 = n.subscribe("motor_distance_speed1", 1000, commandCallback1);
    ros::Subscriber sub2 = n.subscribe("motor_distance_speed2", 1000, commandCallback2);

    

    // Publicador para monitorear la velocidad del motor 1
    ros::Publisher speed_pub1 = n.advertise<std_msgs::Int32>("motor1_speed_feedback", 1000);
    ros::Publisher speed_pub2 = n.advertise<std_msgs::Int32>("motor2_speed_feedback", 1000);

    ros::Rate loop_rate(10);  // Frecuencia de 10Hz

    while (ros::ok()) {
        uint8_t status1, status2;
        bool valid1, valid2;

        int32_t motor_speed1 = roboclaw.ReadEncM1(0x80, &status1, &valid1);  // Lee la velocidad de M1
        int32_t motor_speed2 = roboclaw.ReadEncM2(0x80, &status2, &valid2);  // Lee la velocidad de M2

        if (valid2) {
            std_msgs::Int32 speed_msg2;
            speed_msg2.data = motor_speed2;
            speed_pub2.publish(speed_msg2);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M2");
        }

        if (valid1) {
            std_msgs::Int32 speed_msg1;
            speed_msg1.data = motor_speed1;
            speed_pub1.publish(speed_msg1);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M1");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
