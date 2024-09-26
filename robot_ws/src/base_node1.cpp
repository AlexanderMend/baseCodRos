#include "ros/ros.h"
#include "roboclaw.hpp"  // Asegúrate de que está correctamente incluido
#include "std_msgs/Float64.h"

// Inicializamos el controlador Roboclaw con el puerto y la tasa de baudios correcta
Roboclaw roboclaw("/dev/ttyACM0", 115200);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh;

    // Parámetros PID
    float Kp1 = 4.57566, Ki1 = 0.96595, Kd1 = 0.0;
    float Kp2 = 4.8033, Ki2 = 0.97178, Kd2 = 0.0;
    uint32_t qpps1 = 4312;  // Pulsos por segundo
    uint32_t qpps2 = 4500;
    // Configurar el PID de velocidad para ambos motores
    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);
    roboclaw.SetM2VelocityPID(0x80, Kp2, Ki2, Kd2, qpps2);

    int32_t speedM1 = 0; // Velocidad deseada motor M1
    int32_t speedM2 = 0; // Velocidad deseada motor M2

    // Controlar la velocidad de los motores
    roboclaw.SpeedM1(0x80, speedM1);
    roboclaw.SpeedM2(0x80, speedM2);

    ros::spin();
    return 0;
}

