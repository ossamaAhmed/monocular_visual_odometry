function [newpts, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points
% so that their centroid is at the origin and their mean distance from
% the origin is sqrt(2).
%
% Usage:   [newpts, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   newpts -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, newpts = T*pts
%

num_points = size(pts,2);

% Convert homogeneous coordinates to Euclidean coordinates (pixels)
pts = pts ./ repmat( pts(3,:),3,1);

% Centroid (Euclidean coordinates)
mu = mean(pts(1:2,:),2);

% Average distance or root mean squared distance of centered points
% It does not matter too much which criterion to use. Both improve the
% numerical conditioning of the Fundamental matrix estimation problem.
pts_centered = pts(1:2,:)-repmat(mu,1,num_points);
% Option 1: RMS distance
sigma = sqrt( mean(sum(pts_centered.^2)) );
% Option 2: average distance
% sigma = mean( sqrt(sum(pts_centered.^2)) );

s = sqrt(2) / sigma;

T = [s 0 -s*mu(1);
    0 s -s*mu(2);
    0 0 1];

newpts = T*pts;

end