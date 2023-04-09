%% 创建UR5机器人
robot = createUR5();
%% Traj
referQ = zeros(1,6);
keypoint_T1 = transl(-0.2, 0.6 ,0.3) * trotx(-pi/2);
keypoint_T2 = transl(0.2, 0.6 ,0.3) * trotx(-pi/2);
keypoint_T3 = transl(0.2, 0.6 ,0.7) * trotx(-pi/2);
keypoint_T4 = transl(-0.2, 0.6 ,0.7) * trotx(-pi/2);
%% 逆解
inter_num = 11;
waypoints = zeros(4*(inter_num-1),6);
index = 0;

x_sequence = linspace(keypoint_T1(1,4), keypoint_T2(1,4), inter_num);
y_sequence = linspace(keypoint_T1(2,4), keypoint_T2(2,4), inter_num);
z_sequence = linspace(keypoint_T1(3,4), keypoint_T2(3,4), inter_num);
for i=1:inter_num-1
    thisT = keypoint_T1;
    thisT(1:3, 4) = [x_sequence(i); y_sequence(i); z_sequence(i)];
    newQ = UR5ikineP(robot, thisT, referQ);
    index =  index + 1;
    waypoints(index, :) = newQ;
    referQ = newQ;
end

x_sequence = linspace(keypoint_T2(1,4), keypoint_T3(1,4), inter_num);
y_sequence = linspace(keypoint_T2(2,4), keypoint_T3(2,4), inter_num);
z_sequence = linspace(keypoint_T2(3,4), keypoint_T3(3,4), inter_num);
for i=1:inter_num-1
    thisT = keypoint_T2;
    thisT(1:3, 4) = [x_sequence(i); y_sequence(i); z_sequence(i)];
    newQ = UR5ikineP(robot, thisT, referQ);
    index =  index + 1;
    waypoints(index, :) = newQ;
    referQ = newQ;
end

x_sequence = linspace(keypoint_T3(1,4), keypoint_T4(1,4), inter_num);
y_sequence = linspace(keypoint_T3(2,4), keypoint_T4(2,4), inter_num);
z_sequence = linspace(keypoint_T3(3,4), keypoint_T4(3,4), inter_num);
for i=1:inter_num-1
    thisT = keypoint_T3;
    thisT(1:3, 4) = [x_sequence(i); y_sequence(i); z_sequence(i)];
    newQ = UR5ikineP(robot, thisT, referQ);
    index =  index + 1;
    waypoints(index, :) = newQ;
    referQ = newQ;
end

x_sequence = linspace(keypoint_T4(1,4), keypoint_T1(1,4), inter_num);
y_sequence = linspace(keypoint_T4(2,4), keypoint_T1(2,4), inter_num);
z_sequence = linspace(keypoint_T4(3,4), keypoint_T1(3,4), inter_num);
for i=1:inter_num-1
    thisT = keypoint_T4;
    thisT(1:3, 4) = [x_sequence(i); y_sequence(i); z_sequence(i)];
    newQ = UR5ikineP(robot, thisT, referQ);
    index =  index + 1;
    waypoints(index, :) = newQ;
    referQ = newQ;
end

%% 演示
hold on
for i=1:4*(inter_num-1)
    q = waypoints(i,:);
    robot.plot(q);
end

