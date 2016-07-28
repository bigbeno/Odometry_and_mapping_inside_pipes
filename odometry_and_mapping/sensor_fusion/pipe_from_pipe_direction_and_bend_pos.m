
function [X, Y, Z] = pipe_from_pipe_direction_and_bend_pos(pipe_direction, point1, point2)
    

    r = 0.05; %[m]
    [X,Y,Z] = cylinder(r);
    Z = Z*0.6-0.3;
    
    top = [X(1,:); Y(1,:); Z(1,:)];
    bottom = [X(2,:); Y(2,:); Z(2,:)];  
    
    if (pipe_direction(3)==0)
        v = [0 0 1]';
    else
        x = randn; y = randn; z = -(pipe_direction(1)*x + pipe_direction(2)*y)/pipe_direction(3);
        v = [x y z]'; v = v/norm(v);
    end
    R = [v, cross(pipe_direction, v), pipe_direction];
    if ~((det(R)-1)<=10*eps && (norm(R)-1)<=10*eps)
        disp('ERROR IN R COMPUTATION');
        pause;
    end
    top = R*top;
    bottom = R*bottom;



    %Translate accoridng to pipe start and end

    centroid_top = mean(top, 2);
    centroid_bottom = mean(bottom, 2);
    
    dtransl = [point1; 0] - [centroid_top(1:2);0];
    top =  top + repmat(dtransl,1,length(top));
    
    d_top_bottom = [point1; 0]-centroid_bottom;
    d_top_wanted_bottom = pipe_direction*dot([point2;0] -[point1; 0],pipe_direction);
    dtransl = d_top_bottom+d_top_wanted_bottom;
    bottom = bottom + repmat(dtransl,1,length(bottom));
    
%     dtransl = [point2; 0] - [centroid_bottom(1:2);0];
%     dot(dtransl, pipe_direction)
%     dtransl
%     dtransl = pipe_direction * dot(dtransl, pipe_direction);    
%     bottom =  top + bottom + repmat(dtransl,1,length(bottom));
    
    
    X = [top(1,:); bottom(1,:)];
    Y = [top(2,:); bottom(2,:)];
    Z = [top(3,:); bottom(3,:)];
    
end