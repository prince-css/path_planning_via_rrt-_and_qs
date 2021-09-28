clear all;
clc

a1=4.5;
a2=7.0;
a3=8.5;     % 4.6
a4=7.0;     % 5.0
a5=10.5;    % 4.1
%d3=2.5;    %2.5  % 5
n=1;
while (n<=20000)
    theta1(n)=(180-0).*rand(1,1) + 0;
    theta2(n)=(90-(-90)).*rand(1,1) + (-90);
    d3(n)=(5-0).*rand(1,1) + 0;
    t1=theta1(n)*pi/180;
    t2=theta2(n)*pi/180;
    
    pt = [
        t1 0 a2 a1;
        t2 pi a4 a3;
        0 0 0 a5+d3(n)
    ];
    row=size(pt,1);
    col=size(pt,2);
    
    for i=1:1:row
            H{i} = [
                cos(pt(i,1)) -sin(pt(i,1))*cos(pt(i,2)) sin(pt(i,1))*sin(pt(i,2)) pt(i,3)*cos(pt(i,1));
                sin(pt(i,1)) cos(pt(i,1))*cos(pt(i,2)) -cos(pt(i,1))*sin(pt(i,2)) pt(i,3)*sin(pt(i,1));
                0.0 sin(pt(i,2)) cos(pt(i,2)) pt(i,4);
                0.0 0.0 0.0 1.0
                ];
    end
    H0_2 = H{1}*H{2};
    H0_3 = H0_2*H{3};
    cartesian_coord{n}=(H0_3(1:3,4))';
    
    X(n)=cartesian_coord{n}(1,1);
    Y(n)=cartesian_coord{n}(1,2);
    Z(n)=cartesian_coord{n}(1,3);
    n=n+1;
end   

%plot3(theta1,theta2,d3,'.r')
plot3(X,Y,Z,'.r')
hold on

%path planning
alpha=[0:1:180];
Xp=sqrt(2)*cos(alpha)+7;
Yp=abs(sqrt(2)*sin(alpha))+7;
Zp=zeros(1,181) ;
plot3(Xp,Yp,Zp,'.b')