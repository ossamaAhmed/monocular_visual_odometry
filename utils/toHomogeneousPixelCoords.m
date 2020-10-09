function hom_pixels = toHomogeneousPixelCoords(pixels)
% Takes pixels [N x 2]
hom_pixels = [pixels, ones(size(pixels,1), 1)];
end

