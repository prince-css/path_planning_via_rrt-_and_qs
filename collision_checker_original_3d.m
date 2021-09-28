function touched=collision_checker_original_3d(XX,tri,theta1,theta2, d3)
    touched=false;
    
    [xp, yp, zp]=forward_kine_3d(theta1, theta2, d3);
    for i=1:1:length(tri)
%     for i=1:2:size(cart_obs,1)
%         if(i >= length(cart_obs))
%             break;
%         end
%         x=[
%            cart_obs(i,1);
%            cart_obs(i,1);
%            cart_obs(i+1,1);
%            cart_obs(i+1,1);
%            cart_obs(i,1);
%            cart_obs(i,1);
%            cart_obs(i+1,1);
%            cart_obs(i+1,1);
%            (cart_obs(i,1)+cart_obs(i+1,1))/2;
%            (cart_obs(i,1)+cart_obs(i+1,1))/2;
%            ];
%         y=[
%            cart_obs(i+1,2);
%            cart_obs(i,2);
%            cart_obs(i,2);
%            cart_obs(i+1,2);
%            cart_obs(i+1,2);
%            cart_obs(i,2);
%            cart_obs(i,2);
%            cart_obs(i+1,2);
%            (cart_obs(i,2)+cart_obs(i+1,2))/2;
%            (cart_obs(i,2)+cart_obs(i+1,2))/2;
%            ];
%         z=[
%            cart_obs(i,3);
%            cart_obs(i,3);
%            cart_obs(i,3);
%            cart_obs(i,3);
%            cart_obs(i+1,3);
%            cart_obs(i+1,3);
%            cart_obs(i+1,3);
%            cart_obs(i+1,3);
%            cart_obs(i+1,3);
%            cart_obs(i,3);
%            ];
%         X=[x y z];
%         tri = delaunayn(XX); % Generate delaunay triangulization
%         %tetramesh(tri,XX);
        tn = tsearchn(XX{i}, tri{i}, [xp, yp, zp]); % Determine which triangle point is within
        IsInside = ~isnan(tn); % Convert to logical vector
        if(IsInside==1)
            touched=true;
            break;
        end
    end
    if(theta1 > 180 || theta1 < 0 || theta2 < -90 || theta2 > 90 || d3 > 2.5 || d3 < 0)
           touched=true;
           %fprintf('la');
    end
    %fprintf('ne');
    %disp(touched);
end