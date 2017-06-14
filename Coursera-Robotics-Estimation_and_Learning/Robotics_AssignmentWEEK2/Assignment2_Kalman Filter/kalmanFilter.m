function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
%   observe data: [z] = zx, zy. so the transmission matrix C is 

    %% Place parameters like covarainces, etc. here:
    P = eye(4);
    R = eye(2);
    A = eye(4); %state transmission matrix
    A(1,3) = t - previous_t;
    A(2,4) = t - previous_t;
    C = [1, 0, 0, 0;
        0, 1, 0, 0;]; %observation transmission matrix
%     sigma_m = diag([1e15,1e15,1e17,1e17]); %state noise
    sigma_m = [1e8, 0, 1e10, 0;
               0,  1e8, 0, 1e10;
               1e8, 0, 1e4, 0;
               0, 1e8, 0, 1e4;];
%     sigma_m = [1e15, 0, 1e15, 0;
%                0,  1e15, 0, 1e15;
%                1e15, 0, 1e17, 0;
%                0, 1e15, 0, 1e17;];
%     sigma_o = diag([1e8, 1e8]); %observation noise
    sigma_o = [1e8, 0;
                0, 1e8;];
%     sigma_o = [1e8, 0;
%                 0, 1e8;];
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 1e10 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    
    P = A * param.P * A' + sigma_m;
    R = C * param.P * C' + sigma_o;
    
    K = P * C' / (R + C * P * C');
    K
    
    z = [x, y];
    state = (A * state' + K * (z' - C * A * state'))';
    param.P = P - K * C * P;

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / (t - previous_t);
    vy = (y - state(2)) / (t - previous_t);
%     vx = state(3);
%     vy = state(4);
    % Predict 330ms into the future
    predictx = state(1) + vx * 0.330;
    predicty = state(2) + vy * 0.330;
    % State is a four dimensional element
%     state(1) = predictx;
%     state(2) = predicty;
%     state(3) = vx;
%     state(4) = vy;
    return;
end
