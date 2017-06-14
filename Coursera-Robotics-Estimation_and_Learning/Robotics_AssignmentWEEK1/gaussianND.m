function [pdf] = gaussianND(X, mu, sigma)
%GUASSIAN
%X - Matrix of data points, one per row
%mu -Row vector for the mean
%sigma -  covariance matrix

%get the vector length
n = size(X,2);

%subtract the mean from every data point
meanDiff = bsxfun(@minus, X, mu);

%calculate the multivariate gaussian, sum(X,2) add each row, and get n
%datas
pdf = 1/sqrt((2*pi)^n * det(sigma)) * exp(-1/2 * sum((meanDiff * inv(sigma) .* meanDiff), 2));