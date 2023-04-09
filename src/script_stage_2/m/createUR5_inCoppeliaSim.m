function ur5 = createUR5_inCoppeliaSim(robot_name,base_pose)
%createUR5_inCoppeliaSim 按照CoppeliaSim中UR5机器人的DH表构建机器人
clear L;
%           theta  d       a     alpha 
L(1) = Link([0     0.08916 0       pi/2],'standard') ; 
L(2) = Link([0     0       -0.425   0],'standard') ; 
L(3) = Link([0     0       -0.39225 0],'standard') ; 
L(4) = Link([0     0.10915 0       pi/2],'standard') ; 
L(5) = Link([0     0.09456 0       -pi/2],'standard') ; 
L(6) = Link([0     0.0823  0       0],'standard') ; 

ur5 = SerialLink(L);

if nargin == 0
    ur5.name = 'UR5';
    ur5.base = transl(0,0,0);
elseif nargin == 1
    ur5.name = robot_name;
    ur5.base = transl(0,0,0);
elseif nargin == 2
    ur5.name = robot_name;
    ur5.base = base_pose;
end
end