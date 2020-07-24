function [state] = updateVPs(state, R_cM, optsLPIC)

% Kalman filter parameters
F = eye(6);
H = eye(6);
processNoise = optsLPIC.processNoise;
measurementNoise = optsLPIC.measurementNoise;
Q = eye(6) * processNoise^2;
R = eye(6) * measurementNoise^2;


% covariances
S = R + H * state.P * H.';
K = state.P * H.' * inv(S);


% state correction
VP1 = cartesian2spherical(R_cM(:,1));
VP2 = cartesian2spherical(R_cM(:,2));
VP3 = cartesian2spherical(R_cM(:,3));
z = [VP1; VP2; VP3];
z = correctEulerAngleDiscontinuity(z, state.X);
state.X = state.X + K * (z - H * state.X);
state.X = maintainEulerAngleRange(state.X);


% covariance correction
state.P = (eye(6) - K * H) * state.P;
state.P = 0.5 * state.P + 0.5 * state.P.';


end


function [z_new] = correctEulerAngleDiscontinuity(z, stateX)


% check discontinuity occur
for k = 1:size(z,1)
    if (abs(z(k) - stateX(k)) > pi)
        if (z(k) <= 0)
            z(k) = z(k) + 2*pi;
        elseif (z(k) > 0)
            z(k) = z(k) - 2*pi;
        end
    end
end


% return corrected euler angle
z_new = z;


end

