function [q] = UR5ikine(ur5, T)
%UR5ikine 专为UR机器人所写的逆解函数
% ur5: SerialLink object
% T: End Frame, 4x4 matrix

singularity_tolerance = 0.00001;

d1=ur5.links(1).d;
a2=ur5.links(2).a;
a3=ur5.links(3).a;
d4=ur5.links(4).d;
d5=ur5.links(5).d;
d6=ur5.links(6).d;

Tq = ur5.base.T \ T; 

nx=Tq(1,1);ny=Tq(2,1);nz=Tq(3,1);
ox=Tq(1,2);oy=Tq(2,2);oz=Tq(3,2);
ax=Tq(1,3);ay=Tq(2,3);az=Tq(3,3);
px=Tq(1,4);py=Tq(2,4);pz=Tq(3,4);

q = zeros(8,6);
%% SOLVE Theta 1
temp_A = px - d6*ax;
temp_B = d6*ay-py;
theta1_set0xx = atan2(temp_A*d4 - temp_B * sqrt(temp_A^2 + temp_B^2 - d4^2), temp_B * d4 + temp_A * sqrt(temp_A^2 + temp_B^2 - d4^2));
theta1_set1xx = atan2(temp_A*d4 + temp_B * sqrt(temp_A^2 + temp_B^2 - d4^2), temp_B * d4 - temp_A * sqrt(temp_A^2 + temp_B^2 - d4^2));

%% SOLVE Theta 5
cos_theta5_set0xx = ax * sin(theta1_set0xx) - ay * cos(theta1_set0xx);
cos_theta5_set1xx = ax * sin(theta1_set1xx) - ay * cos(theta1_set1xx);
theta5_set00x = atan2(sqrt(1-cos_theta5_set0xx^2), cos_theta5_set0xx);
theta5_set01x = atan2(-sqrt(1-cos_theta5_set0xx^2), cos_theta5_set0xx);
theta5_set10x = atan2(sqrt(1-cos_theta5_set1xx^2), cos_theta5_set1xx);
theta5_set11x = atan2(-sqrt(1-cos_theta5_set1xx^2), cos_theta5_set1xx);

if abs(sin(theta5_set00x)) < singularity_tolerance || abs(sin(theta5_set01x)) < singularity_tolerance || abs(sin(theta5_set10x)) < singularity_tolerance || abs(sin(theta5_set11x)) < singularity_tolerance
    disp("[Error] sin_theta5 = 0, 机器人处于奇异位置\n")
    q = nan;
    return;
end

%% SLOVE Theta 6
sin_theta6_set00x = (oy * cos(theta1_set0xx) - ox * sin(theta1_set0xx))/sin(theta5_set00x);
cos_theta6_set00x = (nx * sin(theta1_set0xx) - ny * cos(theta1_set0xx))/sin(theta5_set00x);
theta6_set00x = atan2(sin_theta6_set00x, cos_theta6_set00x);

sin_theta6_set01x = (oy * cos(theta1_set0xx) - ox * sin(theta1_set0xx))/sin(theta5_set01x);
cos_theta6_set01x = (nx * sin(theta1_set0xx) - ny * cos(theta1_set0xx))/sin(theta5_set01x);
theta6_set01x = atan2(sin_theta6_set01x, cos_theta6_set01x);

sin_theta6_set10x = (oy * cos(theta1_set1xx) - ox * sin(theta1_set1xx))/sin(theta5_set10x);
cos_theta6_set10x = (nx * sin(theta1_set1xx) - ny * cos(theta1_set1xx))/sin(theta5_set10x);
theta6_set10x = atan2(sin_theta6_set10x, cos_theta6_set10x);

sin_theta6_set11x = (oy * cos(theta1_set1xx) - ox * sin(theta1_set1xx))/sin(theta5_set11x);
cos_theta6_set11x = (nx * sin(theta1_set1xx) - ny * cos(theta1_set1xx))/sin(theta5_set11x);
theta6_set11x = atan2(sin_theta6_set11x, cos_theta6_set11x);

%% STEP 1 END

q(1,:) = [theta1_set0xx, nan, nan, nan, theta5_set00x, theta6_set00x];
q(2,:) = [theta1_set0xx, nan, nan, nan, theta5_set00x, theta6_set00x];
q(3,:) = [theta1_set0xx, nan, nan, nan, theta5_set01x, theta6_set01x];
q(4,:) = [theta1_set0xx, nan, nan, nan, theta5_set01x, theta6_set01x];
q(5,:) = [theta1_set1xx, nan, nan, nan, theta5_set10x, theta6_set10x];
q(6,:) = [theta1_set1xx, nan, nan, nan, theta5_set10x, theta6_set10x];
q(7,:) = [theta1_set1xx, nan, nan, nan, theta5_set11x, theta6_set11x];
q(8,:) = [theta1_set1xx, nan, nan, nan, theta5_set11x, theta6_set11x];

%% SLOVE Theta 2, 3, 4 together

% set 00x
temp_m14 ...
    = nx * d5 * cos(theta1_set0xx) * sin(theta6_set00x) ...
    + ny * d5 * sin(theta1_set0xx) * sin(theta6_set00x) ...
    + ox * d5 * cos(theta1_set0xx) * cos(theta6_set00x) ...
    + oy * d5 * sin(theta1_set0xx) * cos(theta6_set00x) ...
    + (px - ax * d6) * cos(theta1_set0xx) ...
    + (py - ay * d6) * sin(theta1_set0xx);
temp_m24 = nz * d5 * sin(theta6_set00x) + oz * d5 * cos(theta6_set00x)...
    - az * d6 + pz - d1;
temp_C = (temp_m14^2 + temp_m24^2 + a2^2 - a3^2) / (2 * a2);

if temp_C^2 > (temp_m14^2 + temp_m24^2)
    disp("[Error] 存在一种无解情况\n");
else
    sin_theta2_set000 = temp_m24 * temp_C - temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2);
    cos_theta2_set000 = temp_m14 * temp_C + temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2);
    theta2_set000 = atan2(sin_theta2_set000, cos_theta2_set000);
    sin_theta2_set001 = temp_m24 * temp_C + temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2);
    cos_theta2_set001 = temp_m14 * temp_C - temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2);
    theta2_set001 = atan2(sin_theta2_set001, cos_theta2_set001);
    
    
    theta3_set000 = atan2((temp_m24-a2 * sin(theta2_set000)) / a3, (temp_m14-a2 * cos(theta2_set000)) / a3) - theta2_set000;
    theta3_set001 = atan2((temp_m24-a2 * sin(theta2_set001)) / a3, (temp_m14-a2 * cos(theta2_set001)) / a3) - theta2_set001;
    
    temp_m11 ...
        = nx * cos(theta1_set0xx) * cos(theta5_set00x) * cos(theta6_set00x) ...
        + ny * sin(theta1_set0xx) * cos(theta5_set00x) * cos(theta6_set00x) ...
        - ox * cos(theta1_set0xx) * cos(theta5_set00x) * sin(theta6_set00x) ...
        - oy * sin(theta1_set0xx) * cos(theta5_set00x) * sin(theta6_set00x) ...
        - ax * cos(theta1_set0xx) * sin(theta5_set00x) ...
        - ay * sin(theta1_set0xx) * sin(theta5_set00x);
    temp_m12 ...
        = nz * cos(theta5_set00x) * cos(theta6_set00x) ...
        - oz * cos(theta5_set00x) * sin(theta6_set00x) ...
        - az * sin(theta5_set00x);
    theta4_set000 = atan2(temp_m12, temp_m11) - theta2_set000 - theta3_set000;
    theta4_set001 = atan2(temp_m12, temp_m11) - theta2_set001 - theta3_set001;

    q(1,2:4) = [theta2_set000, theta3_set000, theta4_set000];
    q(2,2:4) = [theta2_set001, theta3_set001, theta4_set001];
end
% set 01x
temp_m14 ...
    = nx * d5 * cos(theta1_set0xx) * sin(theta6_set01x) ...
    + ny * d5 * sin(theta1_set0xx) * sin(theta6_set01x) ...
    + ox * d5 * cos(theta1_set0xx) * cos(theta6_set01x) ...
    + oy * d5 * sin(theta1_set0xx) * cos(theta6_set01x) ...
    + (px - ax * d6) * cos(theta1_set0xx) ...
    + (py - ay * d6) * sin(theta1_set0xx);
temp_m24 = nz * d5 * sin(theta6_set01x) + oz * d5 * cos(theta6_set01x)...
    - az * d6 + pz - d1;
temp_C = (temp_m14^2 + temp_m24^2 + a2^2 - a3^2) / (2 * a2);

if temp_C^2 > (temp_m14^2 + temp_m24^2)
    disp("[Error] 存在一种无解情况\n");
else
    theta2_set010 = atan2(...
        temp_m24 * temp_C - temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C + temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    theta2_set011 = atan2(...
        temp_m24 * temp_C + temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C - temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    
    theta3_set010 = atan2((temp_m24-a2 * sin(theta2_set010)) / a3, (temp_m14-a2 * cos(theta2_set010)) / a3) - theta2_set010;
    theta3_set011 = atan2((temp_m24-a2 * sin(theta2_set011)) / a3, (temp_m14-a2 * cos(theta2_set011)) / a3) - theta2_set011;
    
    temp_m11 ...
        = nx * cos(theta1_set0xx) * cos(theta5_set01x) * cos(theta6_set01x) ...
        + ny * sin(theta1_set0xx) * cos(theta5_set01x) * cos(theta6_set01x) ...
        - ox * cos(theta1_set0xx) * cos(theta5_set01x) * sin(theta6_set01x) ...
        - oy * sin(theta1_set0xx) * cos(theta5_set01x) * sin(theta6_set01x) ...
        - ax * cos(theta1_set0xx) * sin(theta5_set01x) ...
        - ay * sin(theta1_set0xx) * sin(theta5_set01x);
    temp_m12 ...
        = nz * cos(theta5_set01x) * cos(theta6_set01x) ...
        - oz * cos(theta5_set01x) * sin(theta6_set01x) ...
        - az * sin(theta5_set01x);
    
    theta4_set010 = atan2(temp_m12, temp_m11) - theta2_set010 - theta3_set010;
    theta4_set011 = atan2(temp_m12, temp_m11) - theta2_set011 - theta3_set011;
    
    q(3,2:4) = [theta2_set010, theta3_set010, theta4_set010];
    q(4,2:4) = [theta2_set011, theta3_set011, theta4_set011];
end

% set 10x
temp_m14 ...
    = nx * d5 * cos(theta1_set1xx) * sin(theta6_set10x) ...
    + ny * d5 * sin(theta1_set1xx) * sin(theta6_set10x) ...
    + ox * d5 * cos(theta1_set1xx) * cos(theta6_set10x) ...
    + oy * d5 * sin(theta1_set1xx) * cos(theta6_set10x) ...
    + (px - ax * d6) * cos(theta1_set1xx) ...
    + (py - ay * d6) * sin(theta1_set1xx);
temp_m24 = nz * d5 * sin(theta6_set10x) + oz * d5 * cos(theta6_set10x)...
    - az * d6 + pz - d1;
temp_C = (temp_m14^2 + temp_m24^2 + a2^2 - a3^2) / (2 * a2);

if temp_C^2 > (temp_m14^2 + temp_m24^2)
    disp("[Error] 存在一种无解情况\n");
else
    theta2_set100 = atan2(...
        temp_m24 * temp_C - temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C + temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    theta2_set101 = atan2(...
        temp_m24 * temp_C + temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C - temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    
    theta3_set100 = atan2((temp_m24-a2 * sin(theta2_set100)) / a3, (temp_m14-a2 * cos(theta2_set100)) / a3) - theta2_set100;
    theta3_set101 = atan2((temp_m24-a2 * sin(theta2_set101)) / a3, (temp_m14-a2 * cos(theta2_set101)) / a3) - theta2_set101;
    
    temp_m11 ...
        = nx * cos(theta1_set1xx) * cos(theta5_set10x) * cos(theta6_set10x) ...
        + ny * sin(theta1_set1xx) * cos(theta5_set10x) * cos(theta6_set10x) ...
        - ox * cos(theta1_set1xx) * cos(theta5_set10x) * sin(theta6_set10x) ...
        - oy * sin(theta1_set1xx) * cos(theta5_set10x) * sin(theta6_set10x) ...
        - ax * cos(theta1_set1xx) * sin(theta5_set10x) ...
        - ay * sin(theta1_set1xx) * sin(theta5_set10x);
    temp_m12 ...
        = nz * cos(theta5_set10x) * cos(theta6_set10x) ...
        - oz * cos(theta5_set10x) * sin(theta6_set10x) ...
        - az * sin(theta5_set10x);
    
    theta4_set100 = atan2(temp_m12, temp_m11) - theta2_set100 - theta3_set100;
    theta4_set101 = atan2(temp_m12, temp_m11) - theta2_set101 - theta3_set101;

    q(5,2:4) = [theta2_set100, theta3_set100, theta4_set100];
    q(6,2:4) = [theta2_set101, theta3_set101, theta4_set101];
end

% set 11x
temp_m14 ...
    = nx * d5 * cos(theta1_set1xx) * sin(theta6_set11x) ...
    + ny * d5 * sin(theta1_set1xx) * sin(theta6_set11x) ...
    + ox * d5 * cos(theta1_set1xx) * cos(theta6_set11x) ...
    + oy * d5 * sin(theta1_set1xx) * cos(theta6_set11x) ...
    + (px - ax * d6) * cos(theta1_set1xx) ...
    + (py - ay * d6) * sin(theta1_set1xx);
temp_m24 = nz * d5 * sin(theta6_set11x) + oz * d5 * cos(theta6_set11x)...
    - az * d6 + pz - d1;
temp_C = (temp_m14^2 + temp_m24^2 + a2^2 - a3^2) / (2 * a2);

if temp_C^2 > (temp_m14^2 + temp_m24^2)
    disp("[Error] 存在一种无解情况\n");
else
    theta2_set110 = atan2(...
        temp_m24 * temp_C - temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C + temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    theta2_set111 = atan2(...
        temp_m24 * temp_C + temp_m14 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2),...
        temp_m14 * temp_C - temp_m24 * sqrt(temp_m14^2 + temp_m24^2 - temp_C^2));
    
    theta3_set110 = atan2((temp_m24-a2 * sin(theta2_set110)) / a3, (temp_m14-a2 * cos(theta2_set110)) / a3) - theta2_set110;
    theta3_set111 = atan2((temp_m24-a2 * sin(theta2_set111)) / a3, (temp_m14-a2 * cos(theta2_set111)) / a3) - theta2_set111;
    
    temp_m11 ...
        = nx * cos(theta1_set1xx) * cos(theta5_set11x) * cos(theta6_set11x) ...
        + ny * sin(theta1_set1xx) * cos(theta5_set11x) * cos(theta6_set11x) ...
        - ox * cos(theta1_set1xx) * cos(theta5_set11x) * sin(theta6_set11x) ...
        - oy * sin(theta1_set1xx) * cos(theta5_set11x) * sin(theta6_set11x) ...
        - ax * cos(theta1_set1xx) * sin(theta5_set11x) ...
        - ay * sin(theta1_set1xx) * sin(theta5_set11x);
    temp_m12 ...
        = nz * cos(theta5_set11x) * cos(theta6_set11x) ...
        - oz * cos(theta5_set11x) * sin(theta6_set11x) ...
        - az * sin(theta5_set11x);
    
    theta4_set110 = atan2(temp_m12, temp_m11) - theta2_set110 - theta3_set110;
    theta4_set111 = atan2(temp_m12, temp_m11) - theta2_set111 - theta3_set111;

    q(7,2:4) = [theta2_set110, theta3_set110, theta4_set110];
    q(8,2:4) = [theta2_set111, theta3_set111, theta4_set111];
end
%% COLLECTION
% q(1,:) = [theta1_set0xx, theta2_set000, theta3_set000, theta4_set000, theta5_set00x, theta6_set00x];
% q(2,:) = [theta1_set0xx, theta2_set001, theta3_set001, theta4_set001, theta5_set00x, theta6_set00x];
% q(3,:) = [theta1_set0xx, theta2_set010, theta3_set010, theta4_set010, theta5_set01x, theta6_set01x];
% q(4,:) = [theta1_set0xx, theta2_set011, theta3_set011, theta4_set011, theta5_set01x, theta6_set01x];
% q(5,:) = [theta1_set1xx, theta2_set100, theta3_set100, theta4_set100, theta5_set10x, theta6_set10x];
% q(6,:) = [theta1_set1xx, theta2_set101, theta3_set101, theta4_set101, theta5_set10x, theta6_set10x];
% q(7,:) = [theta1_set1xx, theta2_set110, theta3_set110, theta4_set110, theta5_set11x, theta6_set11x];
% q(8,:) = [theta1_set1xx, theta2_set111, theta3_set111, theta4_set111, theta5_set11x, theta6_set11x];
end