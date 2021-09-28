clear all;
clc

a1=4.5;
a2=7.0;
a3=4.6;
a4=5.0;
a5=4.1;
allX=[];
allY=[];
allZ=[];
alltheta1=[];
alltheta2=[];
alld3=[];
%%%%%%%%%%%%circular path planning
%(10,2,0) to (-10,2,0)
%(6,8,0) to (8,6,0)

%alpha=[0:1:180];
%X=sqrt(2)*cos(alpha)+7;
%Y=abs(sqrt(2)*sin(alpha))+7;
%Z=zeros(1,181) ;

%%%%%%%%%%%Straight line path planning
%from (-9,4,0) to (-4,10,0)
segments=[[-10 4];[-4 10];[4 10];[10 4]];
for i=1:1:length(segments)
    if i==length(segments)
        break
    end
    
    x_o=segments(i,1)
    y_o=segments(i,2)
    z_o=0;
    x_1=segments(i+1,1)
    y_1=segments(i+1,2)
    z_1=0;
    t=[0:0.02:1];
    X=x_o+(x_1-x_o)*t;
    Y=y_o+(y_1-y_o)*t;
    Z=z_o+(z_1-z_o)*t;
    
    n=1;
    
    while (n<=length(t))
        inRoot=X(n)^2+Y(n)^2;
        r1=sqrt(inRoot);
        fi1=acosd((a2^2+r1^2-a4^2)/(2*a2*r1));
        fi2=acosd((a2^2+a4^2-r1^2)/(2*a2*a4));
        fi3=atand(Y(n)/X(n));
        if(X(n)>=0)
            theta1(n)=fi1+fi3;
            theta2(n)=fi2-180;
        elseif(X(n)<0)
            theta1(n)=180+(fi3-fi1);
            theta2(n)=180-fi2;
            
        end
    d3(n)=(a1+a3)-(Z(n)+a5);
    n=n+1;
    end
    allX=[allX,X];
    allY=[allY,Y];
    allZ=[allZ,Z];
    alltheta1=[alltheta1,theta1];
    alltheta2=[alltheta2,theta2];
    alld3=[alld3,d3];
end
sum(alltheta1>180)
sum(alltheta1<0)
sum(alltheta2<-90)
sum(alltheta2>90)
subplot(1,2,1)
plot3(allX,allY,allZ,'.k')
subplot(1,2,2)
plot3(alltheta1,alltheta2,alld3,'.r')

%% path planning in joint space

x_0=10;
y_0=2;
z_0=0;
x_1=-5;
y_1=8;
z_1=0;

a1=4.5;
a2=7.0;
a3=4.6;
a4=5.0;
a5=4.1;

X(1)=x_0;
Y(1)=y_0;
Z(1)=z_0;
X(2)=x_1;
Y(2)=y_1;
Z(2)=z_1;
for n=1:1:2
inRoot=X(n)^2+Y(n)^2;
    r1=sqrt(inRoot);
    fi1=acosd((a2^2+r1^2-a4^2)/(2*a2*r1));
    fi2=acosd((a2^2+a4^2-r1^2)/(2*a2*a4));
    fi3=atand(Y(n)/X(n));
    if(X(n)>=0)
        theta1(n)=fi1+fi3
        theta2(n)=fi2-180
    elseif(X(n)<0)
        theta1(n)=180+(fi3-fi1)
        theta2(n)=180-fi2   
    end
    d3(n)=(a1+a3)-(Z(n)+a5)
end
t=0:0.02:1;
theta1_con=theta1(1)+(theta1(2)-theta1(1))*t;
theta2_con=theta2(1)+(theta2(2)-theta2(1))*t;
d3_con=d3(1)+(d3(2)-d3(1))*t;
subplot(1,2,1);
plot3(theta1_con,theta2_con,d3_con)
for n=1:1:length(t)
    t1=theta1_con(n)*pi/180;
    t2=theta2_con(n)*pi/180;
    pt = [
        t1 0 a2 a1;
        t2 pi a4 a3;
        0 0 0 a5+d3_con(n)
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
    
    X_cart(n)=cartesian_coord{n}(1,1);
    Y_cart(n)=cartesian_coord{n}(1,2);
    Z_cart(n)=cartesian_coord{n}(1,3);
end
subplot(1,2,2);
plot3(X_cart,Y_cart,Z_cart,'r');



