function [state] = predictVPs(state, optsLPIC)

% Kalman filter parameters
F = eye(6);
H = eye(6);
processNoise = optsLPIC.processNoise;
measurementNoise = optsLPIC.measurementNoise;
Q = eye(6) * processNoise^2;
R = eye(6) * measurementNoise^2;


% state propagation
state.X = F * state.X;
state.X = maintainEulerAngleRange(state.X);


% covariance propagation
state.P = F * state.P * F.' + Q;


end



