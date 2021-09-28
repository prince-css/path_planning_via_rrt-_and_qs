clear all;
close all;
clc;

%obstacle coordinates
cart_obs=[
    11 3 3;
    12 3 3;
    13 3 3;
    9 7 3;
    9 8 3;
    9 9 3;
    9 10 3;
    9 11 3;
    6 7 3;
    6 8 3;
    6 9 3;
    6 10 3;
    3 13 3;
    %3 10 3;
    3 11 3; 
    3 12 3; 
    -1 9 3;
    -1 10 3;
    -1 11 3;
    -1 12 3;
    -6 7 1;
    -6 8 1;
    -6 9 3;
    -6 10 3;
    -6 11 3;
    -6 12 3;
    -6 13 3;
    -11 3 3;
    -12 3 3;
    -13 3 3;
    -14 3 3
    ];
%cart_obs=[6 7;0 10;0 11;-3 8;-3 9]
n=1;
%%%%%%%%%%% mapping obstacles to configuration space %%%%%%%%%%
for theta1=0:1:180
   for theta2=-90:1:90
       for d3=0:0.1:2.5
            if collision_checker_original_3d(cart_obs,theta1,theta2,d3)==true
                theta1_obs(n)=theta1;
                theta2_obs(n)=theta2;
                d3_obs(n)=d3;
                n=n+1;
            end
       end
   end
end
axis([0 180 -90 90])
axis square
plot3(theta1_obs, theta2_obs, d3_obs,'.c')
hold on
d=10; % offset radius which defines the length of the links
end_offset=10;












start=[0, -90, 2.5];
ennd=[180, 90, 2.5];
reached=false;
returned=false;
parent_nodes{1}=[0,0,0];
nodes{1}=start; %cell array of all nodes including 'start' and 'end' node
plot3(start(1),start(2),start(3),'sk')
plot3(ennd(1),ennd(2),ennd(3),'ob')
count=1;
i=1;
while(reached ~= true)
    %generating random node coordinates
    node_rand_x=((180-0).*rand(1,1) + 0);
    node_rand_y=((90-(-90)).*rand(1,1) + (-90));
    node_rand_z=((2.5-0).*rand(1,1) + 0);
    node_rand=[node_rand_x,node_rand_y, node_rand_z];
    %plot(node_rand_x,node_rand_y,'*');
    if(collision_checker_original_3d(cart_obs,node_rand_x,node_rand_y, node_rand_z)==true)
        a=1;
        continue
    end
    %plot(node_rand(1),node_rand(2),'.r')
    for node=1:length(nodes)
        %distance from randomly generated node to all nodes to get the
        %nearest node
        inRoot=(node_rand(1)-nodes{node}(1))^2+(node_rand(2)-nodes{node}(2))^2+(node_rand(3)-nodes{node}(3))^2;
        D(node)=sqrt(inRoot);%D is an array of all distance from random point to others
        
        %distance from end node to all nodes to get the nearest node
        inRoot_end=(ennd(1)-nodes{node}(1))^2+(ennd(2)-nodes{node}(2))^2+(ennd(3)-nodes{node}(3))^2;
        D_end(node)=sqrt(inRoot_end);
        
        %when the goal will be reached
        if(D_end(node)<=end_offset)
            reached=true;
            nodes{i+1}=ennd;
            
            % from "D_end" array find the index of the minimum value/distance
            min_D_end_index=find(D_end==min(D_end));  
            
            x_nearest_end=nodes{min_D_end_index};
            parent_nodes{i+1}=x_nearest_end;
            plot3([x_nearest_end(1) ennd(1)], [x_nearest_end(2) ennd(2)], [x_nearest_end(3) ennd(3)],'r')
            
            % tracking back to the start point
            returning_node=ennd;
            while returned ~= true
                % finding the index of the returning_node in main nodes array
                % using this index later on we will find it's corrsponding
                % parent_node that is the node from which it was originated
                for j=1:1:length(nodes)
                    if nodes{j}==returning_node
                        returning_index=j;
                    end
                end              
                parent_node=parent_nodes{returning_index};
                shortest_path{count}=[round(returning_node(1)), round(returning_node(2)), returning_node(3)];
                plot3([returning_node(1) parent_node(1)],[returning_node(2) parent_node(2)],[returning_node(3) parent_node(3)],'b','LineWidth',2)
                returning_node=parent_node;
                if(returning_node== nodes{1})
                    returned=true;
                    break
                end
                count=count+1;
            end
            
            break
        end
        
    end
    if(reached==true)
        break
    end
    min_D_index=find(D==min(D));
    x_nearest=nodes{min_D_index};
    
    % generating a new node within the offset radius of "d"
    % to the direction vector of randomly generated node("node_rand")
    % and from the node("x_nearest") which is closer to the random node
    x_new_x=x_nearest(1)+(d/min(D))*(node_rand(1)-x_nearest(1));
    x_new_y=x_nearest(2)+(d/min(D))*(node_rand(2)-x_nearest(2));
    x_new_z=x_nearest(3)+(d/min(D))*(node_rand(3)-x_nearest(3));
    x_new=[x_new_x, x_new_y, x_new_z];
    %plot(x_new_x,x_new_y,'.');
    % checking wheather the new node and the in-between nodes are out of
    % obstacles or not
    if(collision_checker_original_3d(cart_obs, x_new_x, x_new_y, x_new_z)==true)
        a=1;
        path_cancel=true;
        continue
    else
        path_cancel=false;
        for t=0:0.1:1
            x_inter_x=x_nearest(1)+(x_new_x-x_nearest(1))*t;
            x_inter_y=x_nearest(2)+(x_new_y-x_nearest(2))*t;
            x_inter_z=x_nearest(3)+(x_new_z-x_nearest(3))*t;
            if collision_checker_original_3d(cart_obs, x_inter_x, x_inter_y, x_inter_z)==true
                path_cancel=true;
                break
            end
        end
        if path_cancel==true
           
           continue 
        end
    end
    nodes{i+1}=x_new;
    parent_nodes{i+1}=x_nearest;
    if path_cancel==false
        plot3([x_nearest(1) x_new(1)], [x_nearest(2) x_new(2)], [x_nearest(3) x_new(3)],'r')
    axis square
    end
    i=i+1;
    drawnow 
end
shortest_path{end+1}=start;

shortest_path_flipped=flip(shortest_path,2);
FID = fopen('C:\My_project\pos3d.txt', 'w');
if FID == -1, error('Cannot create file.'); end
fprintf(FID, '%g %g %g\n',shortest_path_flipped{:} );
fclose(FID);














%% 
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
figure
%plot3(theta1,theta2,d3,'.r')
axis([-15 15 -15 15 0 5])
axis square
plot3(X,Y,Z,'.r')
hold on
create_obstacle([11 3 0],[14 4 3])
create_obstacle([9 7 0],[10 11 3])
create_obstacle([6 7 0],[7 10 3])
create_obstacle([3 11 0],[4 14 3])
create_obstacle([-1 10 0],[-2 13 3])
create_obstacle([-6 7 0],[-7 9 1])
create_obstacle([-6 9 0],[-7 14 3])
create_obstacle([-11 3 0],[-14 4 3])

for i=1:length(shortest_path_flipped)
    if i==length(shortest_path_flipped)
        break
    end
    [x1, y1, z1]=forward_kine_3d(shortest_path_flipped{1,i}(1),shortest_path_flipped{1,i}(2),shortest_path_flipped{1,i}(3));
    [x2, y2, z2]=forward_kine_3d(shortest_path_flipped{1,i+1}(1),shortest_path_flipped{1,i+1}(2),shortest_path_flipped{1,i+1}(3));
    plot3([x1 x2],[y1 y2],[z1 z2],'k','LineWidth',2)
    drawnow
end
