%% 测试逆解函数的健壮性
robot = createUR5();

%% 实验数据

for time = 1:100
    Q_true = 2 * pi * (rand(1,6) - 0.5);
    T_true = robot.fkine(Q_true).T;
    Q_cal = UR5ikine(robot, T_true);
    if isnan(Q_cal)
        continue
    end
    for i=1:8
        counter = 0;
        for j = 1:6
            % 剔除8组解中无解的情况
            if isnan(Q_cal(i,j))
                break;
            end
            
        end
        counter = counter + 1;
    end
    if ~counter
        disp("[ERROR] 无解");
    end
end

disp([num2str(time) '次使用无问题'])