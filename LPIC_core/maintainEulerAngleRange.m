function [stateX_new] = maintainEulerAngleRange(stateX)


% check angle range from [-pi,pi] (atan2)
for k = 1:size(stateX,1)
    if (stateX(k) > pi)
        stateX(k) = stateX(k) - 2*pi;
    elseif (stateX(k) < -pi)
        stateX(k) = stateX(k) + 2*pi;
    end
end


% return corrected euler angle
stateX_new = stateX;


end