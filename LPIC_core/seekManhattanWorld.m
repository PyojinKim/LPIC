function [R_cM, vpInfo, planeNormalVector, surfaceNormalVector, surfacePixelPoint] = seekManhattanWorld(imageCurForLine, imageCur, depthCur, cam, optsLPIC)

% assign current parameters
lineDetector = optsLPIC.lineDetector;
lineDescriptor = optsLPIC.lineDescriptor;
lineLength = optsLPIC.lineLength;
K = cam.K_pyramid(:,:,1);
Kinv = inv(K);


%% initialize and seek dominant 1-plane

% plane and surface normal vector
[pNV, ~] = estimatePlaneNormalRANSAC(imageCur, depthCur, cam, optsLPIC);
[sNV, sPP] = estimateSurfaceNormalGradient_mex(imageCur, depthCur, cam, optsLPIC);
surfaceNormalVector = sNV;
surfacePixelPoint = sPP;


% refine plane normal vector
[pNV, isTracked] = trackSinglePlane(pNV, sNV, optsLPIC);
planeNormalVector = pNV;
if (isTracked == 0)
    disp(num2str(imgIdx));
    disp('lost tracking!');
    planeNormalVector = [];
    R_cM = [];
    vpInfo = [];
    return;
end


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
[R_cM, clusteredLinesIdx] = detectOrthogonalLineRANSAC_ODEP(planeNormalVector, lines, Kinv, cam, optsLPIC);
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


% initialize vpInfo
vpInfo = struct('n',{},'line',{},'index',{});
for k = 1:3
    
    % current VP info
    line = linesVP{k};
    numLine = size(line,2);
    vpInfo(k) = struct('n',numLine,'line',line,'index',k);
end


end

