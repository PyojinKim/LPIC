function [state, R_cM] = extractVPs(state)

% return VPs for VO
R_cM_temp = eye(3);
R_cM_temp(:,1) = spherical2cartesian(state.X(1:2));
R_cM_temp(:,2) = spherical2cartesian(state.X(3:4));
R_cM_temp(:,3) = spherical2cartesian(state.X(5:6));


% maintain orthogonality on SO(3)
[U,~,V] = svd([R_cM_temp(:,1), R_cM_temp(:,2), R_cM_temp(:,3)]);
R_cM = U * V';


% apply the result in state
VP1 = cartesian2spherical(R_cM(:,1));
VP2 = cartesian2spherical(R_cM(:,2));
VP3 = cartesian2spherical(R_cM(:,3));
state.X = [VP1; VP2; VP3];
state.X = maintainEulerAngleRange(state.X);


end



