%MATLAB script for velocity inverse kinematics
L(1)=Link([0,0,1,0]);
L(2)=Link([0,0,1,0]);
beta = 0.05;
damping = 0;
scara = SerialLink(L,'name','TLM'); %Serial link object
qi = deg2rad(input("Enter initial joint angles in degrees=")); %inital joint angle vector
n = 1; %iteration value
q = qi; %joint angle vector
joint_angles = q;
g = input("Enter x,y,z positions of goal ="); %fix z as 0 always
v = input("Enter desired end-effector speed=");
e = (fkine(scara,q).t)'; %end effector vector
T = norm(g-e)/v; %calculated time
while norm(g-e)>0.01
    j = jacob0(scara,q,'trans');
    j = j(1:2,1:2);
    j_inv = pinv(j);
    %matrix = transpose(j)*inv(j*transpose(j)+ damping^2*eye(2));
    matrix = j_inv;
    n = n+1;
    T = norm(g-e)/v;
    vel = (g-e)/T;
    del_t = beta*T;
    omega = matrix*(vel(1:2))';
    q = q+ (omega)'*del_t;
    joint_angles(n,1:2) = q;
    e = (fkine(scara,q).t)';
end

    


