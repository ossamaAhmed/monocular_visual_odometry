function P = toDehomogenousLandmarks(points3d)
    P = points3d./repmat(points3d(4,:),4,1);
    P = P(1:3,:);
end

