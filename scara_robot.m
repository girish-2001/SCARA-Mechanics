%MATLAB script for finding the Jacobian of a Robotic arm using the
%Jacobian-generating vector. 

%defining the DH parameters of the SCARA robot.
L(1)=Link([0,115,210,0]);
L(2)=Link([0,115,210,0]);

scara = SerialLink(L,'name','TLM'); %Serial link object
q = deg2rad(input("Enter joint angles in degrees="));
n = length(q);
J = zeros(3,n); %taking only the translational part of the Jacobian. 
for i = 1:n
    T = fkine(scara,q); %Overall Transformation matrix
    if i==1
        T1 = eye(4);
        T1 = SE3.convert(T1);
    else
        T1 = scara.A(1:i-1,q);
    end
    %T1 = T1.T;
    %T = T.T;
    J(1:3,i) = (cross((T1.R*[0,0,1]')',((T.t)'-(T1.t)')))'; %Equation for jacobian-generating vector
end    
disp(J);
plot(scara,q);




