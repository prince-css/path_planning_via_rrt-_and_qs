function [theta1,theta2,d3]=inv_kine_3d(X,Y,Z)
    a1=4.5;
    a2=7.0;
    a3=8.5;     % 4.6
    a4=7.0;     % 5.0
    a5=10.5;    % 4.1
    %d3=dis;    %2.5  % 5
    inRoot=X^2+Y^2;
    r1=sqrt(inRoot);
    fi1=acosd((a2^2+r1^2-a4^2)/(2*a2*r1));
    fi2=acosd((a2^2+a4^2-r1^2)/(2*a2*a4));
    fi3=atand(Y/X);
    d3=a1+a3-a5-Z
    if(X>=0)
        theta1=fi1+fi3
        theta2=fi2-180
    elseif(X<0)
        theta1=180+(fi3-fi1)
        theta2=180-fi2
    end
end