function [R_cM_final, vpInfo, planeNormalVector, surfaceNormalVector, surfacePixelPoint] = trackManhattanWorld(R_cM_old, pNV_old, imageCurForLine, imageCur, depthCur, cam, optsLPIC)

% assign current parameters
lineDetector = optsLPIC.lineDetector;
lineDescriptor = optsLPIC.lineDescriptor;
lineLength = optsLPIC.lineLength;
K = cam.K_pyramid(:,:,1);
Kinv = inv(K);


%% track (or reinitialize) dominant 1-plane

% track current plane
[sNV, sPP] = estimateSurfaceNormalGradient_mex(imageCur, depthCur, cam, optsLPIC);
[pNV_new, isTracked] = trackSinglePlane(pNV_old, sNV, optsLPIC);
if (isTracked == 0)
    fprintf('Lost tracking! Re-intialize 1-plane normal vector. \n');
    [pNV_new, ~] = estimatePlaneNormalRANSAC(imageCur, depthCur, cam, optsLPIC);
    optsLPIC.minSampleRatio = 0.05;
    [pNV_new, ~] = trackSinglePlane(pNV_new, sNV, optsLPIC);
end
planeNormalVector = pNV_new;
surfaceNormalVector = sNV;
surfacePixelPoint = sPP;


%% find Manhattan frame with 1-plane and 1-line

% line detection
dimageCurForLine = double(imageCurForLine);
if (strcmp(lineDetector,'lsd'))
    [lines, ~] = lsdf(dimageCurForLine, (lineLength^2));
elseif (strcmp(lineDetector,'gpa'))
    [lines, ~] = gpa(imageCurForLine, lineLength);
end
lines = extractUniqueLines(lines, cam);


% do 1-line RANSAC
while (true)
    [R_cM_new, clusteredLinesIdx] = detectOrthogonalLineRANSAC_ODEP(planeNormalVector, lines, Kinv, cam, optsLPIC);
    linesVP = cell(1,3);
    for k = 1:3
        
        % current lines in VPs
        linesInVP = lines(clusteredLinesIdx{k},:);
        numLinesInVP = size(linesInVP,1);
        
        % line clustering for each VP
        line = struct('data',{},'length',{},'centerpt',{},'linenormal',{},'circlenormal',{});
        numLinesCnt = 0;
        for m = 1:numLinesInVP
            [linedata, centerpt, len, ~, linenormal, circlenormal] = roveFeatureGeneration(dimageCurForLine, linesInVP(m,1:4), Kinv, lineDescriptor);
            if (~isempty(linedata))
                numLinesCnt = numLinesCnt+1;
                line(numLinesCnt) = struct('data',linedata,'length',len,'centerpt',centerpt,'linenormal',linenormal,'circlenormal',circlenormal);
            end
        end
        
        % save line clustering results
        linesVP{k} = line;
    end
    
    
    % Manhattan frame matching
    oldMatchingList = zeros(3,1);
    for k = 1:3
        
        % old VP
        vp_old = R_cM_old(:,k);
        
        % new VP
        for m = 1:3
            if (abs(vp_old.' * R_cM_new(:,m)) > cos(deg2rad(5)))
                oldMatchingList(k) = m;
                break;
            end
        end
    end
    
    
    % stop condition
    if (sum(oldMatchingList == 0) == 0)
        break;
    else
        fprintf('Fail 1-line RANSAC. Try again. \n');
    end
end


% new Manhattan frame
R_cM_final = zeros(3,3);
for k = 1:3
    id = oldMatchingList(k);
    
    vp_c = R_cM_new(:,id);
    vp_c_old = R_cM_old(:,k);
    if (acos(vp_c.' * vp_c_old) < deg2rad(5))
        R_cM_final(:,k) = vp_c;
    else
        R_cM_final(:,k) = -vp_c;
    end
end


% initialize vpInfo
vpInfo = struct('n',{},'line',{},'index',{});
for k = 1:3
    id = oldMatchingList(k);
    
    % current VP info
    line = linesVP{id};
    numLine = size(line,2);
    vpInfo(k) = struct('n',numLine,'line',line,'index',k);
end


end