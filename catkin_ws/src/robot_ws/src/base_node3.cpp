#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "roboclaw.hpp"  // Incluye tu librería Roboclaw
#include <cstdlib>

// Inicializa Roboclaw con el puerto adecuado y baudrate
Roboclaw roboclaw("/dev/ttyACM0", 115200)
;
void commandCallback2(const std_msgs::Int8::ConstPtr& msg) {
    int8_t direction = msg->data;  // Recibe el valor de velocidad desde el publicador
    uint8_t address = 0x80;     // Dirección del Roboclaw

    float Kp2 = 4.8033, Ki2 = 0.97178, Kd2 = 0.0;  
    uint32_t qpps2 = 4500;  // Pulsos por segundo

    roboclaw.SetM2VelocityPID(0x80, Kp2, Ki2, Kd2, qpps2);

    if (1) {
        // Mover hacia adelante
        // M2 avanza
        if (roboclaw.ForwardM2(address, abs(direction))) {
        ROS_INFO("Motor M2 set to speed: %d", direction);
        } else {
        ROS_ERROR("Failed to set speed for Motor M2");
        }
    } else {
        // Mover hacia atrás
        if (roboclaw.BackwardM2(address, abs(direction))) {
        ROS_INFO("Motor M2 set to speed: %d", direction);
        } else {
        ROS_ERROR("Failed to set speed for Motor M2");
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
}  

void commandCallback1(const std_msgs::Int8::ConstPtr& msg) {
    int8_t direction = msg->data;  // Recibe el valor de velocidad desde el publicador
    uint8_t address = 0x80;     // Dirección del Roboclaw

    float Kp1 = 2.12612, Ki1 = 0.33859, Kd1 = 0.0;
    const uint32_t qpps1 = 12187;  // Pulsos por segundo

    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);

    if (direction >= 0) {
        // Mover hacia adelante
        // M1 avanza
        if (roboclaw.ForwardM1(address, direction)) {
        ROS_INFO("Motor M1 set to speed: %d", direction);
        } else {
        ROS_ERROR("Failed to set speed for Motor M1");
        }
    } else {
        // Mover hacia atrás
        if (roboclaw.BackwardM1(address, abs(direction))) {
        ROS_INFO("Motor M1 set to speed: %d", direction);
        } else {
        ROS_ERROR("Failed to set speed for Motor M1");
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
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    // No se requiere "connect()" ya que al crear la instancia de Roboclaw, el puerto se abre.

    ros::Subscriber sub1 = n.subscribe("motor_speed1", 1000, commandCallback1);
    ros::Subscriber sub2 = n.subscribe("motor_speed2", 1000, commandCallback2);

    

    // Publicador para monitorear la velocidad del motor 1
    ros::Publisher speed_pub1 = n.advertise<std_msgs::Int8>("motor1_speed_feedback", 1000);
    ros::Publisher speed_pub2 = n.advertise<std_msgs::Int8>("motor2_speed_feedback", 1000);

    ros::Rate loop_rate(10);  // Frecuencia de 10Hz

    while (ros::ok()) {
        uint8_t status;
        bool valid;
        int8_t motor_speed1 = roboclaw.ReadSpeedM1(0x80, &status, &valid);  // Lee la velocidad de M1
        int8_t motor_speed2 = roboclaw.ReadSpeedM2(0x80, &status, &valid);  // Lee la velocidad de M2

        if (valid) {
            std_msgs::Int8 speed_msg2;
            speed_msg2.data = motor_speed2;
            speed_pub2.publish(speed_msg2);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M2");
        }

        if (valid) {
            std_msgs::Int8 speed_msg1;
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
