% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 


load('sigma.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [0.153370393066546,0.653361630645502,0.603170198501544;0.151385895747879,0.525807851496107,0.588237756938583];
sig = [];
sigma{1} = [7.35663578470977e-05,0.000492835702255604,9.74421045235112e-05;0.000492835702255604,0.0133132877426807,0.00421449656964092;9.74421045235112e-05,0.00421449656964092,0.00150637773503719];
sigma{2} = [0.000833512467459435,-0.000484056212938431,-0.000960374452828529;-0.000484056212938431,0.00973146241131898,0.00341568109836483;-0.000960374452828529,0.00341568109836483,0.00251606490290285]
thre = 0.8;
clusters = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
pdf_test = zeros(size(I,1), size(I,2), clusters);
position = zeros(size(I,1), size(I,2));
bw = zeros(size(I,1), size(I,2));
for n = 1:clusters
    for i = 1:size(I,1)
        for j = 1:size(I,2)
            pixel = [I(i,j,1) I(i,j,2) I(i,j,3)];
            hsv = rgb2hsv(double(pixel)/256);
            pdf_test(i,j,n) = gaussianND(hsv, mu(n,:), sigma{n});
            if pdf_test(i,j,n) > thre
                bw(i,j) = 1;
            end
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

bw_biggest = false(size(bw));

CC = bwconncomp(bw);
numPixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
S = regionprops(CC, 'Centroid');

segI = bw_biggest; 
loc = S(idx).Centroid;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
