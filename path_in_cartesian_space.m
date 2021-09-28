clear all;
clc

a1=4.5;
a2=7.0;
a3=8.5;     % 4.6
a4=7.0;     % 5.0
a5=10.5;    % 4.1
%d3=2.5;    %2.5  % 5
n=1;
while (n<=5000)
    theta1(n)=(180-0).*rand(1,1) + 0;
    theta2(n)=(90-(-90)).*rand(1,1) + (-90);
    d3(n)=(2.5-0).*rand(1,1) + 0;
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
axis([-15 15 -15 15 0 5])
axis square
plot3(X,Y,Z,'.r')
hold on
% create_obstacle([11 3 0],[14 4 3])
% create_obstacle([9 7 0],[10 11 3])
% create_obstacle([6 7 0],[7 10 3])
% create_obstacle([3 11 0],[4 14 3])
% create_obstacle([-2 10 0],[-1 13 3])
% create_obstacle([-7 7 0],[-6 9 1])
% create_obstacle([-7 9 0],[-6 14 3])
% create_obstacle([-14 3 0],[-11 4 3])
create_obstacle([11-0.4 3-0.4 0-0.4],[14+0.4 4+0.4 3+0.4])
create_obstacle([9-0.4 7-0.4 0-0.4],[10+0.4 11+0.4 3+0.4])
create_obstacle([6-0.4 7-0.4 0-0.4],[7+0.4 10+0.4 3+0.4])
create_obstacle([3-0.4 11-0.4 0-0.4],[4+0.4 14+0.4 3+0.4])
create_obstacle([-2-0.4 10-0.4 0-0.4],[-1+0.4 13+0.4 3+0.4])
create_obstacle([-7-0.4 7-0.4 0-0.4],[-6+0.4 9+0.4 1+0.4])
create_obstacle([-7-0.4 9-0.4 0-0.4],[-6+0.4 14+0.4 3+0.4])
create_obstacle([-14-0.4 3-0.4 0-0.4],[-11+0.4 4+0.4 3+0.4])

% for i=1:length(shortest_path_flipped)
%     p1=forward_kine_3d(shortest_path_flipped{1,i});
%     p2=forward_kine_3d(shortest_path_flipped{1,i+1});
%     xn=[p1(1) p2(1)];
%     yn=[p1(2) p2(2)];
%     zn=[p1(3) p2(3)];
%     plot3(xn,yn,zn,'.r')
% end