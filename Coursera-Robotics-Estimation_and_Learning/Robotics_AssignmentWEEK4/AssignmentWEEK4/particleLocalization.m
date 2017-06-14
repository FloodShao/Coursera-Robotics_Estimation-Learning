% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 100;                         % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
occ_true = 10;
occ_false = -5;
free_true = 1;
free_false = -5;
cnt =  0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
figure;
j = 2;
sigma_m = diag([5e-2, 5e-2, 0.5]); 

while j <= N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    % random noise added to particles, sigma_x^2, sigma_y^2, sigma_theta^2
    P_idx = zeros(3, M); %Particle 2_D position, [idx_x, idx_y, theta];
    Corelation = zeros(M,1);
    weight = ones(M,1) / M;
    Particle = zeros(3,M);
    
    for i = 1:M    
    % 2) Measurement Update 
%   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)  
        Particle(:,i) = P(:,i) + (ones(1,3) * normrnd(0, sigma_m))';

        %index the particle position
        P_idx(1:2,i) = ceil(Particle(1:2,i) * myResolution) + myOrigin;
        P_idx(3,i) = Particle(3,i);

        if P_idx(1,i) > size(map,1) || P_idx(2,i) > size(map,2) || P_idx(1,i) < 1 || P_idx(2,i) < 1
            Particle(:,i) = P(:,i);
            P_idx(1:2,i) = ceil(P(1:2,i) * myResolution) + myOrigin;
        end

        %based on the particle and ranges, find the occ position and
        %convert to index form
        local_position = [ranges(:,j) .* cos(bsxfun(@plus, Particle(3,i), scanAngles)),...
            -ranges(:,j) .* sin(bsxfun(@plus, Particle(3,i), scanAngles))];
        actual_position = bsxfun(@plus, local_position', Particle(1:2, i));
        occ_idx = bsxfun(@plus, ceil(myResolution * actual_position), myOrigin);
        occ_idx(1,(occ_idx(1,:) > size(map,2))) = size(map,2);
        occ_idx(1,(occ_idx(1,:) < 1)) = 1;
        occ_idx(2,(occ_idx(2,:) > size(map,1))) = size(map,1);
        occ_idx(2,(occ_idx(2,:) < 1)) = 1;

       
%   2-2) For each particle, calculate the correlation scores of the particles
        occ = sub2ind(size(map), occ_idx(2,:), occ_idx(1,:)); %convert the (x,y) to image use tuple (y,x)
        Corelation(i) = Corelation(i) + occ_true * sum(map(occ) > 1.4) + occ_false * sum((map(occ) < 1));
%         for k = 1:size(occ_idx,2)
%             [freex, freey] = bresenham(P_idx(1,i), P_idx(2,i), occ_idx(1,k), occ_idx(2,k));
%             free = sub2ind(size(map), freey, freex);
%             Corelation(i) = Corelation(i) + free_true * sum(map(free) < 0) + free_false * sum((map(free) > 1));
%         end

    end
    Corelation = Corelation - min(Corelation);%to make Corelation all positive
%   2-3) Update the particle weights
    weight = weight .* Corelation; 
    %scale weight
    weight = weight / sum(weight);
    
    %   2-4) Choose the best particle to update the pose
    [mW, mWi] = max(weight);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    num_effective = (sum(weight))^2 /sum(weight.^2);
    if num_effective < 0.4*M

        P = repmat(Particle(:,mWi), [1,M]);
        myPose(:,j) = Particle(:,mWi);
        sigma_m = diag([0.4*abs(cos(Particle(3,mWi))), 0.4*abs(sin(Particle(3,mWi))), 0.5]);
        
        fprintf('The %d point\n cnt = %d\n', j, cnt);
        cnt = 0;
    else 
        if cnt > 150
            myPose(:,j) = P(:,1);
            j = j+1;
%             sigma_m = diag([0.5*abs(cos(Particle(3,mWi))), 0.5*abs(sin(Particle(3,mWi))), 0.5]);
            sigma_m = diag([0.5*abs(cos(myPose(3,j-1))), 0.5*abs(sin(myPose(3,j-1))), 0.5]);
            cnt = 0;
        elseif cnt > 100
%             sigma_m = diag([1.5*abs(cos(Particle(3,mWi))), 1.5*abs(sin(Particle(3,mWi))), 1]);
            sigma_m = diag([abs(cos(myPose(3,j-1))), abs(sin(myPose(3,j-1))), 1]);
%         elseif cnt > 50
% %             sigma_m = diag([abs(cos(Particle(3,mWi))), abs(sin(Particle(3,mWi))), 1]);
%             sigma_m = diag([abs(cos(myPose(3,j-1))), abs(sin(myPose(3,j-1))), 1]);
        else
%             sigma_m = diag([0.8*abs(cos(Particle(3,1))), 0.1*abs(cos(Particle(3,mWi))*sin(Particle(3,mWi))), 0;
%                             0.1*abs(cos(Particle(3,mWi))*sin(Particle(3,mWi))), 0.8*abs(sin(Particle(3,mWi))), 0;
%                             0,0,1]);
%             sigma_m = diag([abs(cos(P(3,1))), abs(sin(P(3,1))), 1]);  
%             sigma_m = diag([abs(cos(Particle(3,mWi))), abs(sin(Particle(3,mWi))), 0.5]);
            sigma_m = diag([0.8*abs(cos(myPose(3,j-1))), 0.8*abs(sin(myPose(3,j-1))), 0.5]);
        end
        cnt = cnt+1;
        j = j-1;
    end
    
    j = j+1;
    
    % 4) Visualize the pose on the map as needed
   plot(P_idx(1,mWi), P_idx(2,mWi), 'rx');
   hold on;
   

end

hold off;
end

