function [X,Y,Z]=forward_kine_3d(theta1,theta2,dis)
    a1=4.5;
    a2=7.0;
    a3=8.5;     % 4.6
    a4=7.0;     % 5.0
    a5=10.5;    % 4.1
    d3=dis;     % 2.5  % 5
    t1=theta1*pi/180;
    t2=theta2*pi/180;
    pt = [
        t1 0 a2 a1;
        t2 pi a4 a3;
        0 0 0 a5+d3
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
    H0_3 = H0_2*H{3}
    cartesian_coord=(H0_3(1:3,4))';
    
    X=(cartesian_coord(1,1));
    Y=(cartesian_coord(1,2));
    Z=(cartesian_coord(1,3));
    
    test1=[cos(t1) -sin(t1) 0 a2*cos(t1);
           sin(t1) cos(t1) 0 a2*sin(t1);
           0 0 1 a1;
           0 0 0 1];
    test2=[cos(t2) sin(t2) 0 a4*cos(t2);
           sin(t2) -cos(t2) 0 a4*sin(t2);
           0 0 -1 a3;
           0 0 0 1];
    test3=[1 0 0 0;
           0 1 0 0;
           0 0 1 a5+d3;
           0 0 0 1;
        ];
    test=test1*test2*test3
    
end