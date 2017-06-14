% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
 myResol = param.resol;
% the initial map size in pixels
 myMap = zeros(param.size);
% the origin of the map in pixels, in resolution form
 myorigin = param.origin; 
% 
% 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,

      
    % Find grids hit by the rays (in the gird map coordinate)
    local_position = [ranges(:,j) .* cos(bsxfun(@plus, pose(3,j), scanAngles)) ,...
        -ranges(:,j) .* sin(bsxfun(@plus, pose(3,j), scanAngles))];
    actual_position = bsxfun(@plus, local_position', pose(1:2, j));
    idx_position = bsxfun(@plus, ceil(myResol * actual_position), myorigin);
%     idx_position = [idx_position(2,:); idx_position(1,:)];
    idx_pose = bsxfun(@plus, ceil(myResol * pose(1:2,j)), myorigin);

  
    % Find occupied-measurement cells and free-measurement cells
    occ = sub2ind(size(myMap), idx_position(2,:), idx_position(1,:));
    myMap(occ) = myMap(occ) + lo_occ;

    for i = 1:(size(idx_position,2))
       [freex, freey] =  bresenham(idx_pose(1), idx_pose(2), idx_position(1,i), idx_position(2,i));
       free = sub2ind(size(myMap), freey, freex); 
       myMap(free) = myMap(free) - lo_free;
    end

end
    
    % Saturate the log-odd values
    myMap(myMap <= lo_min) = lo_min;
    myMap(myMap >= lo_max) = lo_max;

    % Visualize the map as needed
    figure;
    imagesc(myMap);
    colormap('gray'); axis equal;
    
end

