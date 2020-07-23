function [voteSumTotal, clusteredLineIdx] = computeOrthogonalDistance(R_cM, lineEndPixelPoints, centerPixelPoint, lineLength, cam, optsLAPO)

% parameters for orthogonal distance with end points
lineInlierThreshold = optsLAPO.lineInlierThreshold;
angleWeight = optsLAPO.angleWeight;
lengthWeight = optsLAPO.lengthWeight;
minLineNum = optsLAPO.minLineNum;
K = cam.K_pyramid(:,:,1);


% pre-defined variables
numLines = size(lineEndPixelPoints,1);
voteSumTotal = zeros(1,3);
clusteredLineIdx = cell(1,3);
maxLineLength = max(lineLength);


% adjust minimum number of lines
if (numLines <= 15)
    minLineNum = 1;
end


% orthogonal distance with end points
for k = 1:3
    
    % skip VP from plane normal vector
    if (k == 1)
        continue;
    end
    
    
    % initial parameters
    VP = R_cM(:,k);
    VP_n = VP ./ VP(3);
    VP_p = K * VP_n;
    voteSum = 0;
    lineIdx = zeros(numLines,1);
    
    
    % find line inlier
    for m = 1:numLines
        
        % general line equation
        centerpt = centerPixelPoint(m,:);
        lineModel = estimateLineModel(VP_p(1:2), centerpt);
        A = lineModel(1);
        B = lineModel(2);
        C = lineModel(3);
        
        
        % orthogonal distance
        linedata = lineEndPixelPoints(m,:);
        denominator = sqrt(A^2 + B^2);
        d_pt1 = (abs(A*linedata(1) + B*linedata(2) + C) / denominator);
        d_pt2 = (abs(A*linedata(3) + B*linedata(4) + C) / denominator);
        d_error = (d_pt1 + d_pt2) / 2;
        if (d_error <= lineInlierThreshold)
            voteSum = voteSum + angleWeight * (1-(d_error/lineInlierThreshold)) + lengthWeight * (lineLength(m)/maxLineLength);
            lineIdx(m) = 1;
        end
    end
    
    
    % save the results
    if (sum(lineIdx) >= minLineNum)
        voteSumTotal(k) = voteSum + sum(lineIdx);
        clusteredLineIdx{k} = find(lineIdx == 1);
        lineEndPixelPoints(clusteredLineIdx{k},:) = NaN;
    else
        voteSumTotal(k) = 0;
        clusteredLineIdx{k} = [];
    end
end


end





