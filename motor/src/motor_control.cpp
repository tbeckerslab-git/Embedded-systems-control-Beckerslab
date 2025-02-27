#include "../include/motor_control.h"
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <limits>

// Default setting
#define DXL_ID                  1           // Dynamixel ID: 1
#define BAUDRATE                57600
#define DEVICENAME              "/dev/ttyUSB0"
#define MINIMUM_POSITION_LIMIT  0
#define MAXIMUM_POSITION_LIMIT  4095

// Protocol version
#define PROTOCOL_VERSION        2.0

// Addresses
#define ADDR_MX_TORQUE_ENABLE       64
#define ADDR_MX_GOAL_POSITION       116
#define ADDR_MX_PRESENT_POSITION    132

// Initialize PortHandler and PacketHandler instances
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Esc key checker
bool isKeyPressed() {
    int ch;
    int oldf = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch == 27) {  // 27 =ASCII for esc
        return true;
    }
    if (ch != EOF) {
        ungetc(ch, stdin);
    }
    return false;
}

int main()
{
    int initResult = initializeDynamixel();
    if (initResult != 0)
    {
        fprintf(stderr, "Failed to initialize Dynamixel.");
        return initResult;
    }

    while (true)
    {
        std::cout << "Enter desired position (0-4095) or type 'exit' to quit: ";
        std::string input;
        std::cin >> input;

        if (input == "exit") {
            break;  // Exit the loop
        }

        try {
            int desiredPosition = std::stoi(input);
            
            // Check if the desired position is within the valid range
            if (desiredPosition < 0 || desiredPosition > 4095) {
                std::cout << "Invalid position. Please enter a position within the range 0-4095." << std::endl;
                continue;
            }

            setGoalPosition(desiredPosition);
            usleep(100000); // Add a delay to control the speed (microseconds)

        } catch(const std::exception&) {
            std::cout << "Invalid input. Please enter a numeric position within the range 0-4095." << std::endl;
        }
    }

    return 0;
}

int initializeDynamixel()
{
    // Open port
    if (!portHandler->openPort())
    {
        printf("Failed to open the port!\n");
        return -1;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE))
    {
        printf("Failed to change the baudrate!\n");
        return -2;
    }

    // Enable Dynamixel Torque
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 1);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return -3;
    }
    else
    {
        uint8_t dxl_error = 0;
        packetHandler->getRxPacketError(dxl_error);
        if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -4;
        }
    }

    return 0;
}

void setGoalPosition(int position)
{
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, position);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else
    {
        uint8_t dxl_error = 0;
        packetHandler->getRxPacketError(dxl_error);
        if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    }
}
