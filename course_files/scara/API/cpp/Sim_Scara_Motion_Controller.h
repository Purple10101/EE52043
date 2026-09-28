#pragma once
#include <iostream>
#include <winsock2.h>
#include <cstdint>
#include <cstring>
#pragma comment(lib, "ws2_32.lib")

class Sim_Scara_Motion_Controller {
private:
    SOCKET Sock;
    WSADATA wsa;

public:
    Sim_Scara_Motion_Controller();
    ~Sim_Scara_Motion_Controller();

    template <typename T>
    void SendCommand(const T& cmd) {
        send(Sock, reinterpret_cast<const char*>(&cmd), sizeof(T), 0);

        int32_t successFlag;
        int totalReceived = 0;
        int size = sizeof(int32_t);
        char* ptr = reinterpret_cast<char*>(&successFlag);

        while (totalReceived < size) {
            int bytes = recv(Sock,
                             ptr + totalReceived,
                             size - totalReceived,
                             0);

            if (bytes <= 0) {
                std::cout << "Receive failed\n";
                return;
            }
            totalReceived += bytes;
        }
        std::cout << "Success flag: " << successFlag << std::endl;
    }

    // Robot commands
    void MoveJoint(int Axis, float Angle, int T);
    void MoveJoints(float Angle0, float Angle1, float Angle2, float Angle3, int T);
    void MoveCoord(float X, float Y, float Z_Angle, float Z, int T);
    void Initialise(float Link1, float Link2, float Z_Min, float Z_Max, float SettlingTime, float P_0, float I_0, float D_0, float P_1, float I_1, float D_1);
};