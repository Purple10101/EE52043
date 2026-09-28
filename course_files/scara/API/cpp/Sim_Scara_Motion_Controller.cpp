#include "Sim_Scara_Motion_Controller.h"

Sim_Scara_Motion_Controller::Sim_Scara_Motion_Controller() {
    struct sockaddr_in server;

    WSAStartup(MAKEWORD(2,2), &wsa);

    Sock = socket(AF_INET , SOCK_STREAM , 0);

    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons(9000);

    if (connect(Sock , (struct sockaddr *)&server , sizeof(server)) < 0) {
        std::cout << "Connection error\n";
    } else {
        std::cout << "Connected!\n";
    }
}

Sim_Scara_Motion_Controller::~Sim_Scara_Motion_Controller() {
    closesocket(Sock);
    WSACleanup();
}

void Sim_Scara_Motion_Controller::MoveJoint(int Axis, float Angle, int T) {
    #pragma pack(push, 1)
    struct CommandMoveJoint {
        int32_t a, axis;
        float angle, time;
        int32_t zeros[8];
    };
    #pragma pack(pop)

    CommandMoveJoint cmd{};
    cmd.a = 3;
    cmd.axis = Axis;
    cmd.angle = Angle;
    cmd.time = T;

    SendCommand(cmd);
}

void Sim_Scara_Motion_Controller::MoveJoints(float Angle0, float Angle1, float Angle2, float Angle3, int T) {
    #pragma pack(push, 1)
    struct CommandMoveJoints {
        int32_t a;
        float angle0, angle1, angle2, angle3, time;
        int32_t zeros[6];
    };
    #pragma pack(pop)

    CommandMoveJoints cmd{};
    cmd.a = 4;
    cmd.angle0 = Angle0;
    cmd.angle1 = Angle1;
    cmd.angle2 = Angle2;
    cmd.angle3 = Angle3;
    cmd.time = T;

    SendCommand(cmd);
}

void Sim_Scara_Motion_Controller::MoveCoord(float X, float Y, float Z_Angle, float Z, int T) {
    #pragma pack(push, 1)
    struct CommandMoveJoints {
        int32_t a;
        float X, Y, Z_Angle, Z, time;
        int32_t zeros[6];
    };
    #pragma pack(pop)

    CommandMoveJoints cmd{};
    cmd.a = 5;
    cmd.X= X;
    cmd.Y = Y;
    cmd.Z_Angle = Z_Angle;
    cmd.Z = Z;
    cmd.time = T;

    SendCommand(cmd);
}

void Sim_Scara_Motion_Controller::Initialise(float Link1, float Link2, float Z_Min, float Z_Max, float SettlingTime, float P_0, float I_0, float D_0, float P_1, float I_1, float D_1) {
    #pragma pack(push, 1)
    struct CommandMoveJoints {
        int32_t a;
        float Link1, Link2, Z_Min, Z_Max, SettlingTime, P_0, I_0, D_0, P_1, I_1, D_1;
    };
    #pragma pack(pop)

    CommandMoveJoints cmd{};
    cmd.a = 1;
    cmd.Link1 = Link1;
    cmd.Link2 = Link2;
    cmd.Z_Min = Z_Min;
    cmd.Z_Max = Z_Max;
    cmd.SettlingTime = SettlingTime;
    cmd.P_0 = P_0;
    cmd.I_0 = I_0;
    cmd.D_0 = D_0;
    cmd.P_1 = P_1;
    cmd.I_1 = I_1;
    cmd.D_1 = D_1;
    
    SendCommand(cmd);
}