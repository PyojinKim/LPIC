function [X_cart] = spherical2cartesian(X_sph)

theta = X_sph(1);
phi = X_sph(2);

x = cos(phi)*cos(theta);
y = cos(phi)*sin(theta);
z = sin(phi);

X_cart = [x; y; z];


end