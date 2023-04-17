clear all;

%% ---- Grid Setup ---- %%
grid on
view(15,15)
axis equal
title("Task 2c - Stacking")
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([-0.3 0.4])
xlabel(['x']);
ylabel(['y']);
zlabel(['z']);


%% ---- Setup ---- %%

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end


%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE              = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION              = 116; 

ADDR_PRO_PRESENT_POSITION           = 132; 
ADDR_PRO_OPERATING_MODE             = 11;

ADDR_PROFILE_VELOCITY               = 112;
ADDR_PROFILE_ACCELERATION           = 108;

ADDR_DRIVE_MODE = 10;


%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION                     = 2.0;          % See which protocol version is used in the Dynamixel


% Default setting
DXL_ID_BASE_SPIN                     = 11;            
DXL_ID_ROT_ONE                       = 12;
DXL_ID_ROT_TWO                       = 13;
DXL_ID_ROT_THREE                     = 14;
DXL_ID_GRIPPER                       = 15;

BAUDRATE                             = 115200;
DEVICENAME                           = '/dev/tty.usbserial-FT5NYB6Z';  
                                            
TORQUE_ENABLE                        = 1;            % Value for enabling the torque
TORQUE_DISABLE                       = 0;            % Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE           = 600;          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE           = 3400;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD          = 20;           % Dynamixel moving status threshold

TIME_BASED_PROFILE                   = 4;

DXL_PROFILE_VELOCITY                 = 10;           % In Drive Mode this is interpreted as a time in ms 
TIME_DURATION                        = 1500;
TIME_ACCEL                           = 850;

ESC_CHARACTER                        = 'e';          % Key for escaping loop

COMM_SUCCESS                         = 0;            % Communication Success result value
COMM_TX_FAIL                         = -1001;        % Communication Tx Failed


%% ------ Opening the Port -------- %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;                                                      % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                                                                       % Dynamixel error
dxl_present_position = 0;                                                            % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


%% ------ Intialising Robot -------- %%

% Put actuator into Position Control Mode
setPositionControlMode(port_num, PROTOCOL_VERSION, ADDR_PRO_OPERATING_MODE, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE);

% Set Robot into Drive Mode with Time Based Profile 
setDriveMode(port_num, PROTOCOL_VERSION, ADDR_DRIVE_MODE, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)

% Enable Dynamixel Torque
enableTorque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE, DXL_ID_BASE_SPIN, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE, DXL_ID_GRIPPER, TORQUE_ENABLE);


%% ------ Comm Port Stuff -------- %%
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end


%% ------ Moving Robot -------- %%
% Set to Intial Position 
moveIntialPos(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION, TIME_DURATION, ...
                                   TIME_ACCEL, DXL_ID_BASE_SPIN,DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE,1294)
pause(3);


%% ------Set Profile Time and Acceleration-----%%
set_time(port_num, PROTOCOL_VERSION, ADDR_PROFILE_VELOCITY, TIME_DURATION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
set_acceleration(port_num, PROTOCOL_VERSION,ADDR_PROFILE_ACCELERATION, TIME_ACCEL, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)


%% ------ Drawing Coordinates -----%%
%     /\ B
%    /  \
%   /    \
%  /      \
% / C      \ A

Z_DIST      = 0.0763;
GAMMA_ANGLE = 365;
PEN_START   = [7.4,7.4,0.0945,GAMMA_ANGLE];
A           = [-7,4,Z_DIST,GAMMA_ANGLE];
B           = [-7,8,Z_DIST,GAMMA_ANGLE];
C           = [-5,6,Z_DIST,GAMMA_ANGLE];
D           = [-7,6,Z_DIST,GAMMA_ANGLE];
PEN_END     = [7.15,7.15,0.09,GAMMA_ANGLE];


%% ------ Angles For Drawing -----%%

radius = 1;
angle = 270;

id_array1 = circle(D(1), D(2), Z_DIST, radius, angle,GAMMA_ANGLE);
id_array2 = straight_horizontal_line(D(1),C(1),C(2),Z_DIST,GAMMA_ANGLE);
id_array3 = diagonal_line(C(1),B(1),C(2),B(2),Z_DIST,GAMMA_ANGLE);
id_array4 = straight_vertical_line(B(1),B(2),A(2),Z_DIST,GAMMA_ANGLE);

angles = [id_array1;id_array2;id_array3;id_array4];

%% ------ Pick Pen Up -------- %%

GRIPPER_ANGLE1 = checkGripperAngle(1400);
GRIPPER_ANGLE2 = checkGripperAngle(1899); % TO DO: Check that the gripper is closed on pen 

pick_pen(GRIPPER_ANGLE1,PEN_START(1),PEN_START(2),PEN_START(3),PEN_START(4), ...
           GRIPPER_ANGLE2, A(1),A(2),A(3),A(4), ...
           port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION,TIME_DURATION, TIME_ACCEL, ...
           DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE);
pause(0.5);

%% ------Set Profile Time and Acceleration-----%%
set_time(port_num, PROTOCOL_VERSION, ADDR_PROFILE_VELOCITY, 1000, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
set_acceleration(port_num, PROTOCOL_VERSION,ADDR_PROFILE_ACCELERATION, 150, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)


%% ------ Starting Position -------- %%
moveGoalPos(GRIPPER_ANGLE2,A(1),A(2),A(3)+0.03,A(4), port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
pause(0.5);
moveGoalPos(GRIPPER_ANGLE2,A(1),A(2),A(3),A(4), port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
set_time(port_num, PROTOCOL_VERSION, ADDR_PROFILE_VELOCITY, 1500, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)   

%% ------ Draw -------- %%   

for k = 1:size(angles,1)
  array = angles(k,:);
 
  SPIN_ANGLE = array(1);
  ROT1_ANGLE = array(2);
  ROT2_ANGLE = array(3); 
  ROT3_ANGLE = array(4);
  
  
  moveToDraw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE, SPIN_ANGLE, ROT1_ANGLE, ROT2_ANGLE, ROT3_ANGLE)
  
  pause(0.6);
end

%% ------ End up Position -------- %%
moveGoalPos(GRIPPER_ANGLE2,C(1),C(2),C(3),C(4), port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
pause(0.5);
moveGoalPos(GRIPPER_ANGLE2,C(1),C(2),C(3)+0.03,C(4), port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
set_time(port_num, PROTOCOL_VERSION, ADDR_PROFILE_VELOCITY, 1500, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)   

%% ------ Drop pen -------- %% 
GRIPPER_ANGLE1 = checkGripperAngle(1899);
GRIPPER_ANGLE2 = checkGripperAngle(1400); % TO DO: Check that the gripper is closed on pen 

pick_pen(GRIPPER_ANGLE1,PEN_END(1),PEN_END(2),PEN_END(3),PEN_END(4), ...
           GRIPPER_ANGLE2, A(1),A(2),A(3),A(4), ...
           port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION,TIME_DURATION, TIME_ACCEL, ...
           DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE);
pause(0.5);

%% ------ Comm Port Stuff -------- %%   
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;

%% ------ Movement Functions -------- %%
function enableTorque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE, DXL_ID_BASE_SPIN, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE, DXL_ID_GRIPPER, TORQUE_ENABLE)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

end

function setPositionControlMode(port_num, PROTOCOL_VERSION, ADDR_PRO_OPERATING_MODE, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PRO_OPERATING_MODE, 3);
end

function setDriveMode(port_num, PROTOCOL_VERSION, ADDR_DRIVE_MODE, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_DRIVE_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_DRIVE_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_DRIVE_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_DRIVE_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_DRIVE_MODE, 4);

end

function  GRIPPER_ANGLE = checkGripperAngle(UNCHECKED_GRIPPER_ANGLE)
    if UNCHECKED_GRIPPER_ANGLE > 2616
        GRIPPER_ANGLE = 2616;
    end 
    
    if UNCHECKED_GRIPPER_ANGLE < 1294
        GRIPPER_ANGLE = 1294;
    end 

    if (UNCHECKED_GRIPPER_ANGLE<=2616) && (UNCHECKED_GRIPPER_ANGLE>=1294)
        GRIPPER_ANGLE = UNCHECKED_GRIPPER_ANGLE;
    end 
end 

function moveIntialPos(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION, TIME_DURATION, TIME_ACCEL, DXL_ID_BASE_SPIN,DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE,GRIPPER_ANGLE)
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PROFILE_VELOCITY, TIME_DURATION);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PRO_GOAL_POSITION, 2048);

   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PROFILE_VELOCITY, TIME_DURATION);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PRO_GOAL_POSITION, 2600); %2535

   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PROFILE_VELOCITY, TIME_DURATION);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PRO_GOAL_POSITION, 2567);

   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PROFILE_VELOCITY, TIME_DURATION);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PRO_GOAL_POSITION, 1034);
  
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PROFILE_VELOCITY, TIME_DURATION);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PRO_GOAL_POSITION, GRIPPER_ANGLE);
end

function move_intermediate_Pos(x_coord,y_coord,z,gamma, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION, TIME_DURATION, TIME_ACCEL, DXL_ID_BASE_SPIN,DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE,GRIPPER_ANGLE)  
   moveGoalPos(GRIPPER_ANGLE, x_coord,y_coord,z+0.06,gamma, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
   
   pause(1);
   
   moveIntialPos(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,ADDR_PROFILE_VELOCITY, ...
       ADDR_PROFILE_ACCELERATION, TIME_DURATION, TIME_ACCEL, DXL_ID_BASE_SPIN,DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE,GRIPPER_ANGLE)
end

function moveGoalPos(GRIPPER_ANGLE,x_coord,y_coord,z,gamma, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
   [SPIN_ANGLE,ROT1_ANGLE,ROT2_ANGLE,ROT3_ANGLE] = kinematics2(x_coord,y_coord,z,gamma);
  
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PRO_GOAL_POSITION, SPIN_ANGLE);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PRO_GOAL_POSITION, ROT3_ANGLE); %2535
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PRO_GOAL_POSITION, ROT2_ANGLE);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PRO_GOAL_POSITION, ROT1_ANGLE);
   
   pause(1);
   
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PRO_GOAL_POSITION, GRIPPER_ANGLE);
end

function moveToDraw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE, SPIN_ANGLE, ROT1_ANGLE, ROT2_ANGLE, ROT3_ANGLE)
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PRO_GOAL_POSITION, SPIN_ANGLE);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PRO_GOAL_POSITION, ROT3_ANGLE); %2535
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PRO_GOAL_POSITION, ROT2_ANGLE);
   write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PRO_GOAL_POSITION, ROT1_ANGLE);
end

function pick_pen(GRIPPER_ANGLE1,x1_coord,y1_coord,z1,gamma1, GRIPPER_ANGLE2, x2_coord,y2_coord,z2,gamma2, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, ADDR_PROFILE_VELOCITY,ADDR_PROFILE_ACCELERATION, TIME_DURATION, TIME_ACCEL, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    % Move to initial position and grab the block
    moveGoalPos(GRIPPER_ANGLE1,x1_coord,y1_coord,z1+0.1,gamma1, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    pause(1);
    moveGoalPos(GRIPPER_ANGLE2,x1_coord,y1_coord,z1,gamma1, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    pause(1);
    %moveGoalPos(GRIPPER_ANGLE2,x1_coord,y1_coord,z1,gamma1, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    move_intermediate_Pos(x1_coord-1,y1_coord-1,z1+0.05,gamma1, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,ADDR_PROFILE_VELOCITY, ...
            ADDR_PROFILE_ACCELERATION, TIME_DURATION, TIME_ACCEL, DXL_ID_BASE_SPIN,DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE,GRIPPER_ANGLE2)   
   
    pause(1);
 
end

function set_time(port_num, PROTOCOL_VERSION, ADDR_PROFILE_VELOCITY, TIME_DURATION, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PROFILE_VELOCITY, TIME_DURATION);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PROFILE_VELOCITY, TIME_DURATION);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PROFILE_VELOCITY, TIME_DURATION);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PROFILE_VELOCITY, TIME_DURATION);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PROFILE_VELOCITY, TIME_DURATION);
end

function set_acceleration(port_num, PROTOCOL_VERSION,ADDR_PROFILE_ACCELERATION, TIME_ACCEL, DXL_ID_BASE_SPIN, DXL_ID_GRIPPER, DXL_ID_ROT_ONE, DXL_ID_ROT_TWO, DXL_ID_ROT_THREE)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE_SPIN, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_THREE, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_TWO, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_ROT_ONE, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_GRIPPER, ADDR_PROFILE_ACCELERATION, TIME_ACCEL);   
end

%% ------ Drawing Functions -------- %%

function id_array = straight_horizontal_line(x1,x2,y1,z,gamma)
    xdiff=abs(x1-x2);
    
    n=xdiff*5;
    xvals = linspace(x1,x2,n);
    [id11, id12, id13, id14] = kinematics2(xvals(1),y1,z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;

    while i<=n
        [b1, b2, b3, b4] =kinematics2(xvals(i),y1,z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        id_array = [id_array;b];
    end
end

function id_array = straight_vertical_line(x1,y1,y2,z,gamma)
    ydiff= abs(y1-y2);
    n=ydiff*5;
    yvals = linspace(y1,y2,n);
    [id11, id12, id13, id14] = kinematics2(x1,yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
   
    while i<=n
        [b1, b2, b3, b4] = kinematics2(x1,yvals(i),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        id_array = [id_array;b];
    end  
end

function id_array = diagonal_line(x1,x2,y1,y2,z,gamma)
    ydiff= abs(y1-y2);
    n=ydiff*5;
    xvals=linspace(x1,x2,n);
    yvals=linspace(y1,y2,n);
    [id11, id12, id13, id14] = kinematics2(xvals(1),yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
    k=2;
    
    while i<=n
        [b1, b2, b3, b4] = kinematics2(xvals(i),yvals(k),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        k=k+1;
        id_array = [id_array;b];
     
    end
end

function id_array = circle(x1,y1,z,r,angle,gamma)
    n=(angle/90)*20;
    theta = linspace(0, angle*pi/180, n); % Define angle range in reverse order
    
    if x1<0
%         xvals = -r*cos(theta)+x1+r; % Define x coordinates with offset and sign change
    xvals = -r*cos(theta)+x1;
    end
    
    if x1>0
%        xvals = r*cos(theta)+x1-r; % Define x coordinates with offset and sign change
    xvals = r*cos(theta)+x1;
    end
   
    yvals = r*sin(theta)+y1+r; % Define y coordinates
    [id11, id12, id13, id14] = kinematics2(xvals(1),yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
    k=2;
   
    while i<=n
        [b1, b2, b3, b4] = kinematics2(xvals(i),yvals(k),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        k=k+1;
        id_array = [id_array;b];
    end  
end


%% ------ Kinematics Functions -------- %%
%---------------- Create DH-Matrix----------------------------------%
function Ti = createDHMatrix(alpha, a, d, thetha)
    Ti = [cosd(thetha) -sind(thetha) 0 a;
          sind(thetha)*cosd(alpha) cosd(thetha)*cosd(alpha) -sind(alpha) -sind(alpha)*d;  
          sind(thetha)*sind(alpha) cosd(thetha)*sind(alpha) cosd(alpha) cosd(alpha)*d;
          0 0 0 1 ];
    
end

%---------------- Forward Kinematics----------------------------------%
function [T01, T02, T03,T04, T05] = doForwardKinematics(spin, rot_1, rot_2, rot_3)
    % DH table
    thetha1 = spin;
    thetha2 = rot_1;
    thetha3 = rot_2;
    thetha4 = rot_3;
    thetha5 = 0;
   
    % angle between axis, measured along X
    alpha = [0 90 0 0 0];  

    % distance between axis measured along X
    a = [0 0 0.130 0.124 0.126 ]; 

    % distance between links, measured along Z
    d = [0.077 0 0 0 0]; 
    
    % angle between links, measured about Z
    thetha =[thetha1 thetha2 thetha3 thetha4 thetha5]; 

    
    
    T1 = createDHMatrix(alpha(1),a(1),d(1),thetha(1));%T_0_1
    T2 = createDHMatrix(alpha(2),a(2),d(2),thetha(2));%T_1_2
    T3 = createDHMatrix(alpha(3),a(3),d(3),thetha(3));
    T4 = createDHMatrix(alpha(4),a(4),d(4),thetha(4));
    T5 = createDHMatrix(alpha(5),a(5),d(5),thetha(5));
    
    T01=T1;
    T02=T1*T2;
    T03=T1*T2*T3;
    T04=T1*T2*T3*T4;
    T05=T1*T2*T3*T4*T5;
    
end

%---------------- Plot forward Kinematics----------------------------------%
function [lines,frame1,frame2,frame3,frame4,frame5]=plotForward(T01, T02, T03, T04, T05)
    origin = [0 0 0];
    base = [T01(1,4) T01(2,4) T01(3,4)];
    shoulder = [T02(1,4) T02(2,4) T02(3,4)];
    elbow = [T03(1,4) T03(2,4) T03(3,4)];
    arm = [T04(1,4) T04(2,4) T04(3,4)];
    wrist = [T05(1,4) T05(2,4) T05(3,4)];

    
    pl0=line([origin(1) base(1)], [origin(2) base(2)], [origin(3) base(3)],'Color','k');
    pl1=line([base(1) shoulder(1)], [base(2) shoulder(2)], [base(3) shoulder(3)],'Color','k');
    pl2=line([shoulder(1) elbow(1)], [shoulder(2) elbow(2)], [shoulder(3) elbow(3)],'Color','k');
    pl3=line([elbow(1) arm(1)], [elbow(2) arm(2)], [elbow(3) arm(3)],'Color','k');
    pl4=line([arm(1) wrist(1)], [arm(2) wrist(2)], [arm(3) wrist(3)],'Color','k');
    pl0.LineWidth = 5;
    pl1.LineWidth = 5;
    pl2.LineWidth = 5;
    pl3.LineWidth = 5;
    pl4.LineWidth = 5;
    
    lines=[pl0,pl1,pl2,pl3,pl4];
    
    % quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,1),T01(2,1),T01(3,1),0.04,'r', 'LineWidth', 2);
    % quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,2),T01(2,2),T01(3,2),0.04,'g', 'LineWidth', 2);
    % quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,3),T01(2,3),T01(3,3),0.04,'b', 'LineWidth', 2);
    
    f1x=quiver3(0,0,0,T01(1,1),T01(2,1),T01(3,1),0.04,'r', 'LineWidth', 2);
    f1y=quiver3(0,0,0,T01(1,2),T01(2,2),T01(3,2),0.04,'g', 'LineWidth', 2);
    f1z=quiver3(0,0,0,T01(1,3),T01(2,3),T01(3,3),0.04,'b', 'LineWidth', 2);
    
    
    
    f2x=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,1),T02(2,1),T02(3,1),0.04,'r', 'LineWidth', 2);
    f2y=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,2),T02(2,2),T02(3,2),0.04,'g', 'LineWidth', 2);
    f2z=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,3),T02(2,3),T02(3,3),0.04,'b', 'LineWidth', 2);
    
    f3x=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,1),T03(2,1),T03(3,1),0.04,'r', 'LineWidth', 2);
    f3y=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,2),T03(2,2),T03(3,2),0.04,'g', 'LineWidth', 2);
    f3z=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,3),T03(2,3),T03(3,3),0.04,'b', 'LineWidth', 2);
    
    f4x=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,1),T04(2,1),T04(3,1),0.04,'r', 'LineWidth', 2);
    f4y=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,2),T04(2,2),T04(3,2),0.04,'g', 'LineWidth', 2);
    f4z=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,3),T04(2,3),T04(3,3),0.04,'b', 'LineWidth', 2);
    
    f5x=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,1),T05(2,1),T05(3,1),0.04,'r', 'LineWidth', 2);
    f5y=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,2),T05(2,2),T05(3,2),0.04,'g', 'LineWidth', 2);
    f5z=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,3),T05(2,3),T05(3,3),0.04,'b', 'LineWidth', 2); 
    
    frame1 =[f1x,f1y,f1z];
    frame2 = [f2x,f2y,f2z];
    frame3 = [f3x,f3y,f3z];
    frame4 = [f4x,f4y,f4z];
    frame5 = [f5x,f5y,f5z];
    
end

%---------------- Inverse Kinematics----------------------------------%
function [spin_angle, theta_one, theta_two, theta_three] = doInverseKinematics(target_x, target_y, target_z,gamma)
    l_one = 0.130;
    l_two = 0.124;
    l_three = 0.126;
    %---------------- Spin Angle----------------------------------%
    spin_angle = atan2d(target_y,target_x);
    
    %---------------- Wrist Coordinates----------------------------------%
    r_end = sqrt((target_x^2)+ (target_y^2));
    r = r_end-cosd(gamma)*l_three;
    
    x_wrist = cosd(spin_angle)*r;
    y_wrist = sind(spin_angle)*r;
    z_diff = l_three*sind(gamma);
    
    z_wrist = target_z-z_diff;
    
    target_plane = sqrt((x_wrist^2)+ (y_wrist^2));
    z1 = 0.077;
    pos_z = z_wrist-z1;
    
    %---------------- Theta two----------------------------------%
    cos_theta_two = (((target_plane^2)+(pos_z^2)-(l_one^2)-(l_two^2))/(2*l_one*l_two));
    sin_theta_two(1) = sqrt(1-(cos_theta_two^2));
    sin_theta_two(2) = -sqrt(1-(cos_theta_two^2));
    
    theta_two(1)=atan2d(sin_theta_two(1),cos_theta_two);
    theta_two(2)=atan2d(sin_theta_two(2),cos_theta_two);
    
    %---------------- Theta one----------------------------------%
    k_one = l_one +l_two*cos_theta_two;
    k_two(1) = l_two*sin_theta_two(1);
    k_two(2) = l_two*sin_theta_two(2);
    
    theta_one(1) = atan2d(pos_z,target_plane)-atan2d(k_two(1),k_one);
    theta_one(2) = atan2d(pos_z,target_plane)-atan2d(k_two(2),k_one);
    
    %---------------- Theta Three----------------------------------%
    theta_three(1) = gamma-(theta_two(1)+theta_one(1));
    theta_three(2) = gamma-(theta_two(2)+theta_one(2));

end

%---------------- Delete Robot----------------------------------%
function deleterobo_frame(frame)
    delete(frame(1))
    delete(frame(2))
    delete(frame(3))
end

function deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    delete(line)
    deleterobo_frame(frame1)
    deleterobo_frame(frame2)
    deleterobo_frame(frame3)
    deleterobo_frame(frame4)
    deleterobo_frame(frame5)   
end

%---------------- Flip function(gamma equals 0)----------------------------------%
function [id11,id12,id13,id14]=flip(x,y,z)
    gamma=0;
    [id11,id12,id13,id14]=kinematics2(x,y,z,gamma);
end

%---------------- Run function(gamma equals 270/straight down)----------------------------------%
function [id11,id12,id13,id14]=run(x,y,z)
    gamma=270;
    [id11,id12,id13,id14]=kinematics2(x,y,z,gamma);
end

function [id11,id12,id13,id14]=kinematics2(x_coord,y_coord,z,gamma)
    [x,y] = coord_to_dist(x_coord,y_coord);
    [spin,theta_one,theta_two,theta_three]=doInverseKinematics(x,y,z,gamma);
    %[T01, T02, T03, T04, T05] = doForwardKinematics(spin,theta_one(2),theta_two(2),theta_three(2));
    %hold on
    %plotForward(T01, T02, T03, T04, T05);
    %---------------- Offsets to map model to real----------------------------------%
    offset1=10.62;
    offset2 = 10.62;
    
    spin=spin-90;
    theta_one(2)= 270-theta_one(2)-offset1;
    theta_two(2) = 90-theta_two(2)+offset2;
    theta_three(2) = 180-theta_three(2);
    
    %---------------- Positive angles----------------------------------%
    if spin<0
        spin=spin+360;
    end
    if theta_one(2)<0
        theta_one(2);
        theta_one=theta_one+360;
    end
    if theta_two(2)<0
        theta_two(2);
        theta_two=theta_two+360;
    end
    if theta_three(2)<0
        theta_three(2);
        theta_three=theta_three+360;
    end
    
    id11=(((spin)/360)*4096);
    id12=(theta_one(2)/360)*4096;
    id13=(theta_two(2)/360)*4096;
    id14=(((theta_three(2))/360)*4096);

end

%---------------- Map coordinates to distances----------------------------------%
function [x_dist,y_dist]=coord_to_dist(x_coord,y_coord)
    x_dist = -0.025*x_coord;
    y_dist = -0.025*y_coord;
end






