function [state] = initializeVPs(R_c1M, optsLPIC)


% state for VPs
VP1 = cartesian2spherical(R_c1M(:,1));
VP2 = cartesian2spherical(R_c1M(:,2));
VP3 = cartesian2spherical(R_c1M(:,3));
state.X = [VP1; VP2; VP3];
state.X = maintainEulerAngleRange(state.X);


% covariance for VPs
initialVPAngleNoise = optsLPIC.initialVPAngleNoise;
state.P = eye(6) * initialVPAngleNoise^2;


end



