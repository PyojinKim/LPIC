function [R_cM_final, clusteredLineIdx_final] = detectOrthogonalLineRANSAC_ODEP(planeNormalVector, lines, Kinv, cam, optsLAPO)

% line information for 1-line RANSAC
numLines = size(lines,1);
greatcircleNormal = zeros(numLines,3);
lineEndPixelPoints = zeros(numLines,4);
centerPixelPoint = zeros(numLines,2);
lineLength = zeros(numLines,1);
for k = 1:numLines
    
    % line pixel information
    linedata = lines(k,1:4);
    centerpt = (linedata(1:2) + linedata(3:4))/2;
    length = sqrt((linedata(1)-linedata(3))^2 + (linedata(2)-linedata(4))^2);
    
    % normalized image plane
    ptEnd1_p_d = [linedata(1:2), 1].';
    ptEnd2_p_d = [linedata(3:4), 1].';
    ptEnd1_n_d = Kinv * ptEnd1_p_d;
    ptEnd2_n_d = Kinv * ptEnd2_p_d;
    ptEnd1_n_u = [undistortPts_normal_mex(ptEnd1_n_d(1:2), cam); 1];
    ptEnd2_n_u = [undistortPts_normal_mex(ptEnd2_n_d(1:2), cam); 1];
    
    % normal vector of great circle
    circleNormal = cross(ptEnd1_n_u.', ptEnd2_n_u.');
    circleNormal = circleNormal / norm(circleNormal);
    
    % save the result
    greatcircleNormal(k,:) = circleNormal;
    lineEndPixelPoints(k,:) = linedata;
    centerPixelPoint(k,:) = centerpt;
    lineLength(k) = length;
end


%% 1-line RANSAC

% initialize RANSAC model parameters
totalLineNum = size(lines,1);
sampleLineNum = 1;
ransacMaxIterNum = 1000;
ransacIterNum = 50;
ransacIterCnt = 0;
proximityThreshold = deg2rad(3);

maxVoteSumTotal = 0;
maxVoteSumIdx = [];
isSolutionFound = 0;


% VP1 from plane normal vector
VP1 = planeNormalVector;


% do 1-line RANSAC
while (true)
    
    % sample 1 line feature
    [sampleIdx] = randsample(totalLineNum, sampleLineNum);
    greatcircleNormalSample = greatcircleNormal(sampleIdx,:).';
    
    
    % estimate VP2
    if (abs(acos(dot(VP1, greatcircleNormalSample)) - pi/2) < proximityThreshold)
        continue;
    end
    VP2 = cross(VP1, greatcircleNormalSample);
    VP2 = VP2 / norm(VP2);
    
    
    % estimate VP3
    VP3 = cross(VP1, VP2);
    VP3 = VP3 / norm(VP3);
    
    
    % estimate rotation model parameters
    R_cM_temporary = [VP1, VP2, VP3];
    
    
    % check number of inliers
    [voteSumTotal, clusteredLineIdx] = computeOrthogonalDistance(R_cM_temporary, lineEndPixelPoints, centerPixelPoint, lineLength, cam, optsLAPO);
    
    
    % save the large consensus set
    if (sum(voteSumTotal) >= maxVoteSumTotal)
        maxVoteSumTotal = sum(voteSumTotal);
        maxVoteSumIdx = sampleIdx;
        max_R_cM = R_cM_temporary;
        maxClusteringNum = (size(clusteredLineIdx{1},1) + size(clusteredLineIdx{2},1) + size(clusteredLineIdx{3},1));
        isSolutionFound = 1;
        
        
        % calculate the number of iterations (http://en.wikipedia.org/wiki/RANSAC)
        clusteringRatio = maxClusteringNum / totalLineNum;
        ransacIterNum = ceil(log(0.01)/log(1-(clusteringRatio)^sampleLineNum));
    end
    
    ransacIterCnt = ransacIterCnt + 1;
    if (ransacIterCnt >= ransacIterNum || ransacIterCnt >= ransacMaxIterNum)
        break;
    end
end


% re-formulate 1-line RANSAC result
if (isSolutionFound == 1)
    
    % get clustered lines
    [~, clusteredLineIdx] = computeOrthogonalDistance(max_R_cM, lineEndPixelPoints, centerPixelPoint, lineLength, cam, optsLAPO);
    R_cM_initial = max_R_cM;
    
    
    % refine RANSAC results
    if (isempty(clusteredLineIdx{2}) && isempty(clusteredLineIdx{3}))
        R_cM_final = zeros(3);
        clusteredLineIdx_final = cell(1,3);
    else
        % re-arrange line information for nonlinear optimization
        lineEndPixelPoints_inMF = [];
        centerPixelPoint_inMF = [];
        lineMFLabel_inMF = [];
        for k = 2:3
            
            % current lines in VPs
            linesInVP = lines(clusteredLineIdx{k},:);
            numLinesInVP = size(linesInVP,1);
            
            % line pixel point information
            lineEndPixelPoints_inMF = [lineEndPixelPoints_inMF; lineEndPixelPoints(clusteredLineIdx{k},:)];
            centerPixelPoint_inMF = [centerPixelPoint_inMF; centerPixelPoint(clusteredLineIdx{k},:)];
            lineMFLabel_inMF = [lineMFLabel_inMF; ones(numLinesInVP,1) * k];
        end
        
        
        % run nonlinear optimization using lsqnonlin in Matlab (Levenberg-Marquardt)
        options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter-detailed');
        [vec,resnorm,residuals,exitflag] = lsqnonlin(@(x) orthogonalDistanceResidual(lineEndPixelPoints_inMF,centerPixelPoint_inMF,lineMFLabel_inMF,cam.K,R_cM_initial,planeNormalVector,x),0,[],[],options);
        
        
        % optimal R_cM
        R_cM_final = computeOptimalMF(R_cM_initial, planeNormalVector, vec);
        clusteredLineIdx_final = clusteredLineIdx;
    end
else
    R_cM_final = zeros(3);
    clusteredLineIdx_final = cell(1,3);
end


end

