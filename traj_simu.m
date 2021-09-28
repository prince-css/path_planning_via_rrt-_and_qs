clc;

n=length(shortest_path_flipped)-1;
t=0:2:n*2;%row vector
a=ones(n,4);%n*4 matrix
aa=a;%n*4 matrix
v=ones(1,n+1)*2;% assuming all the via points have same velocity 2
for i=1:1:(n+1)
    p=shortest_path_flipped{i};
    q1(i)=p(1,1);
    q2(i)=p(1,2);
    %via_points(i,1)=q1(i);
    %via_points(i,2)=q2(i);
end
%mstraj(via_points, [2 1], [], [4 1], 0.05, 1)
% cs = spline(t,[0 q1 0]);
% xx = linspace(-4,4,101);
% plot(t,q1,'-');
% oo=ppval(xx,cs);
% q_k,t_k,v_k are given

for k=1:1:n
    %joint 1 :
    T(k)=t(k+1)-t(k);
    
    a(k,1)=q1(k);
    a(k,2)=v(k);
    a(k,3)=(1/T(k))*(((3*(q1(k+1) - q1(k)))/T(k))-2*v(k)-v(k+1));
    a(k,4)=(1/T(k)^2)*((2*(q1(k)-q1(k+1))/T(k))+v(k)+v(k+1));
    
    %joint 2 :
    T(k)=t(k+1)-t(k);
    
    aa(k,1)=q2(k);
    aa(k,2)=v(k);
    aa(k,3)=(1/T(k))*(((3*(q2(k+1) - q2(k)))/T(k))-2*v(k)-v(k+1));
    aa(k,4)=(1/T(k)^2)*((2*(q2(k)-q2(k+1))/T(k))+v(k)+v(k+1));
    
    
    for tt= t(k) : 0.1 : t(k+1)-0.1 % incrementing in each segment by 0.1 sec
        %t_vec=ones(1,length(tt))*t(k);
        
        q1(k)=a(k,1)+a(k,2)*(tt-t(k))+a(k,3)*(tt-t(k)).^2+a(k,4)*(tt-t(k)).^3;
        subplot(3,2,1)
        plot(tt,q1(k),'.b');
        hold on
        
        q1dot(k)=a(k,2)+2*a(k,3)*(tt-t(k))+3*a(k,4)*(tt-t(k)).^2;
        %q1dot(k)=a(k,2);
        subplot(3,2,3)
        plot(tt,q1dot(k),'.b');
        hold on
        
        q1ddot(k)=2*a(k,3)+6*a(k,4)*(tt-t(k));
        %q1ddot(k)=2*a(k,3)+6*a(k,4)*T(k);
        subplot(3,2,5)
        plot(tt,q1ddot(k),'.b');
        hold on 
        
        q2(k)=aa(k,1)+aa(k,2)*(tt-t(k))+aa(k,3)*(tt-t(k)).^2+aa(k,4)*(tt-t(k)).^3;
        subplot(3,2,2)
        plot(tt,q2(k),'.r');
        hold on
        
        q2dot(k)=aa(k,2)+2*aa(k,3)*(tt-t(k))+3*aa(k,4)*(tt-t(k)).^2;
        %q2dot(k)=aa(k,2);
        subplot(3,2,4)
        plot(tt,q2dot(k),'.r');
        hold on
        
        q2ddot(k)=2*aa(k,3)+6*aa(k,4)*(tt-t(k));
        %q2ddot(k)=2*aa(k,3)+6*aa(k,4)*T(k);
        subplot(3,2,6)
        plot(tt,q2ddot(k),'.r');
        hold on
        drawnow
    end
    
end


FID = fopen('C:\My_project\a.txt', 'w');
if FID == -1, error('Cannot create file.'); end
fprintf(FID, '%g %g %g %g\n',a );
fclose(FID);
FID = fopen('C:\My_project\aa.txt', 'w');
if FID == -1, error('Cannot create file.'); end
fprintf(FID, '%g %g %g %g\n',aa );
fclose(FID);
