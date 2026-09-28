classdef scara_motion_controller
    properties
        SerialPort
    end

    methods
        function obj = scara_motion_controller(port)
            obj.SerialPort = port;
        end
        function Initialise(obj, link_1, link_2, z_min, z_max, settling_time, P_0, I_0, D_0, P_1, I_1, D_1)
            data = [ ...
                typecast(int32(1), 'uint8'), ...
                typecast(single([ ...
                link_1, link_2, z_min, z_max, settling_time, ...
                P_0, I_0, D_0, P_1, I_1, D_1 ...
                ]), 'uint8') ...
                ];

            obj.SendCommand(data)
        end
        function MoveJoint(obj, axis, angle,time)
            data = [...
                typecast(int32(3),    'uint8'), ...
                typecast(int32(axis), 'uint8'), ...
                typecast(single(angle),'uint8'), ...
                typecast(single(time), 'uint8'), ...
                typecast(int32(zeros(1,8)), 'uint8') ...
                ];
            
            obj.SendCommand(data)
        end
        function MoveJoints(obj, angle_0, angle_1, angle_2, angle_3, time)
            data = [ ...
                typecast(int32(4), 'uint8'), ...
                typecast(single([angle_0, angle_1, angle_2, angle_3, time]), 'uint8'), ...
                typecast(int32(zeros(1,6)), 'uint8') ...
                ];
            
            obj.SendCommand(data)
        end

        function MoveCoord(obj, x, y, z_angle, z, time)
            data = [ ...
                typecast(int32(5), 'uint8'), ...
                typecast(single([x, y, z_angle, z, time]), 'uint8'), ...
                typecast(int32(zeros(1,6)), 'uint8') ...
                ];
            
            obj.SendCommand(data)
        end

        function AutoCalibrate(obj)
            data = [ ...
                typecast(int32(2), 'uint8'), ...
                typecast(int32(zeros(1,11)), 'uint8') ...
                ];
            
            obj.SendCommand(data)
        end

        function angle = ReadAngle(obj,axis)
            data = [ ...
                typecast(int32(6), 'uint8'), ...
                typecast(int32(axis), 'uint8'), ...
                typecast(int32(zeros(1,10)), 'uint8') ...
                ];
            
            angle = obj.SendCommand(data);
        
        end

        function coords = ReadCoord(obj)
            data = [ ...
                typecast(int32(7), 'uint8'), ...
                typecast(int32(zeros(1,11)), 'uint8') ...
                ];
            
            coords = obj.SendCommand(data);
        
        end

        function out = SendCommand(obj, command)
            channel = serialport(obj.SerialPort,115200,Timeout=60);
            write(channel,command,"uint8");
            disp("sending command...")
            reply = read(channel,1,"int32");
            fprintf("Success Flag: %d\n", reply);
            x = command(1);
            if x == 6
                angle = read(channel, 1, "single");
                out = angle;
            end
            if x == 7
                Coords = read(channel,4,"single");
                out = Coords;
            end
            delete(channel)
        end
    end
end