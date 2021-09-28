clear all;
close all;
clc;


r=35;
axis([0 180 -90 90])
axis square
rectangle('Position',[0,20,40,10],'FaceColor',[0 0 0])
rectangle('Position',[60,50,10,40],'FaceColor',[0 0 0])
rectangle('Position',[110,50,10,40],'FaceColor',[0 0 0])
rectangle('Position',[140,20,40,10],'FaceColor',[0 0 0])
rectangle('Position',[0,-30,40,10],'FaceColor',[0 0 0])
rectangle('Position',[60,-90,10,40],'FaceColor',[0 0 0])
rectangle('Position',[110,-90,10,40],'FaceColor',[0 0 0])
rectangle('Position',[140,-30,40,10],'FaceColor',[0 0 0])
rectangle('Position',[90-r,0-r,r*2,r*2],'FaceColor',[0 0 0])

hold on

d=3;
end_offset=10;
start=[0,-90];
ennd=[180,90];
reached=false;
returned=false;
parent_nodes{1}=[0,0];
nodes{1}=start; %cell array of all nodes including 'start' and 'end' node
plot(start(1),start(2),'sk')
plot(ennd(1),ennd(2),'ob')
i=1;
while(reached ~= true)
    %generating random node coordinates
    node_rand_x=((180-0).*rand(1,1) + 0);
    node_rand_y=((90-(-90)).*rand(1,1) + (-90));
    node_rand=[node_rand_x,node_rand_y];
    if(collision_checker(node_rand_x,node_rand_y)==true)
        a=1
        continue
    end
    %plot(node_rand(1),node_rand(2),'.r')
    for node=1:length(nodes)
        %distance from randomly generated node to all nodes to get the
        %nearest node
        inRoot=(node_rand(1)-nodes{node}(1))^2+(node_rand(2)-nodes{node}(2))^2;
        D(node)=sqrt(inRoot);%D is an array of all distance from random point to others
        
        %distance from end node to all nodes to get the nearest node
        inRoot_end=(ennd(1)-nodes{node}(1))^2+(ennd(2)-nodes{node}(2))^2;
        D_end(node)=sqrt(inRoot_end);
        if(D_end(node)<=end_offset)
            reached=true
            nodes{i+1}=ennd;
            min_D_end_index=find(D_end==min(D_end));
            x_nearest_end=nodes{min_D_end_index};
            parent_nodes{i+1}=x_nearest_end;
            plot([x_nearest_end(1) ennd(1)],[x_nearest_end(2) ennd(2)],'r')
            returning_node=ennd;
            
            while returned ~= true
                for j=1:1:length(nodes)
                    if nodes{j}==returning_node
                        returning_index=j;
                    end
                end              
                parent_node=parent_nodes{returning_index}
                plot([returning_node(1) parent_node(1)],[returning_node(2) parent_node(2)],'b','LineWidth',2)
                returning_node=parent_node;
                if(returning_node== nodes{1})
                    returned=true;
                    break
                end
            end
            
            break
        end
        
    end
    if(reached==true)
        break
    end
    min_D_index=find(D==min(D));
    x_nearest=nodes{min_D_index};
    x_new_x=x_nearest(1)+(d/min(D))*(node_rand(1)-x_nearest(1));
    x_new_y=x_nearest(2)+(d/min(D))*(node_rand(2)-x_nearest(2));
    x_new=[x_new_x,x_new_y];
    if(collision_checker(x_new_x,x_new_y)==true)
        a=1
        continue
%     else
%         for t=0:0.1:1
%             x_inter=x_nearest
%         end
    end
    nodes{i+1}=x_new;
    parent_nodes{i+1}=x_nearest;
    plot([x_nearest(1) x_new(1)],[x_nearest(2) x_new(2)],'r')
    axis square
    i=i+1
    drawnow 
end




