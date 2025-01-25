#include "dynamixel_sdk/dynamixel_sdk.h"  // Use DYNAMIXEL SDK library
#include <signal.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"

//  volatile sig_atomic_t stop;
// void inthand(int signum) {
//     stop = 1;
// }

class MotorTorqueController {
public:
  void motorCallback(const std_msgs::Float64 &msg ){
    // Set goal current (torque)
  auto goal_current = msg.data;  // Adjust the goal current based on your motor and application
  int converted_goal_int = 30;
  printf("Goal current: %d\n", converted_goal_int);

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, 102, goal_current, &dxl_error);  // Address 102 for Goal Current
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  } else {
    printf("Goal current (torque) applied.\n");
  }
}

  MotorTorqueController() {
    ros::Subscriber sub_ = n_.subscribe("/set_torque", 1, &MotorTorqueController::motorCallback, this);
    ros::Publisher pub_ = n_.advertise<std_msgs::Float64>("/torque_status", 10);

    // Initialize PortHandler and PacketHandler instances
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);  // Protocol version 2.0

    // Open the port
    if (portHandler->openPort()) {
      printf("Port opened successfully.\n");
    } else {
      printf("Failed to open the port.\n");
      return;
    }

    // Set the baud rate
    if (portHandler->setBaudRate(baud_rate)) {
      printf("Baud rate set to %d.\n", baud_rate);
    } else {
      printf("Failed to set baud rate.\n");
      return;
    }

    // Enable torque (Torque ON)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 64, 0, &dxl_error);  // Address 64 for Torque Enable
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
      printf("Torque enabled.\n");
    }

    // Set motor to Current Control Mode
    uint8_t current_control_mode = 0;  // Current Control Mode value
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 11, current_control_mode, &dxl_error);  // Address 11 for Operating Mode
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
      printf("Current Control Mode enabled.\n");
    }

    // Set the address and value for current limit
    int current_limit_address = 38;
    int16_t current_limit_value = 1000;  // Set max current limit to around 1000 * 2.69 mA = 2.69 A

    // Write the current limit value
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, current_limit_address, current_limit_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      } else {
        printf("Current Limit has been successfully set.\n");
    }

    // Enable torque (Torque ON)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 64, 1, &dxl_error);  // Address 64 for Torque Enable
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
      printf("Torque enabled.\n");
    }
    

    
  }

  ~MotorTorqueController() {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, 102, 0, &dxl_error);  // Address 102 for Goal Current
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
      printf("Motor stopped.\n");
    }

    // Close port
    portHandler->closePort();
  }  

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  uint8_t dxl_id = 1;  // DYNAMIXEL ID
  int baud_rate = 57600;
  int dxl_comm_result;
  uint8_t dxl_error;
}; // End of class MotorController





int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_actuator");

  MotorTorqueController motorController;

  ros::spin();
 
}

