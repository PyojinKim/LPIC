function [X_sph] = cartesian2spherical(X_cart)

x = X_cart(1);
y = X_cart(2);
z = X_cart(3);

theta = atan2(y, x);
phi = atan2(z, sqrt(x^2+y^2));

X_sph = [theta; phi];


end

