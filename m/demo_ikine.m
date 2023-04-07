%% 创建UR5机器人
robot = createUR5();
%% robot.teach()
%% 实验数据
% Q_true = 2 * pi * (rand(1,6) - 0.5);
% T_true = robot.fkine(Q_true).T;


T_true = transl(-0.3, 0.2 ,0.4);
%% PLOT
% robot.plot(Q_true);

figure
hold on
trplot(T_true, 'frame', 'True', 'color', 'r', 'thick', 4, 'length', 0.5);

%% 逆解
Q_cal = UR5ikine(robot, T_true);
if isnan(Q_cal)
    disp("ERROR， 奇异位置");
end
for i=1:8
    for j = 1:6
        % 剔除8组解中无解的情况
        if isnan(Q_cal(i,j))
            break;
        end
    end
    q = Q_cal(i,:);
    T_cal = robot.fkine(q).T;
    trplot(T_cal, 'frame', ['Cal-' num2str(i)], 'color', 'b', 'length', 0.5 + i * 0.3);
end


