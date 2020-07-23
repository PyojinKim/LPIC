clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

addpath('addon/lsd_1.6');
addpath('addon/lsd_1.6/Matlab');


%% basic setup for LPRVO

% choose the experiment case
% TUM RGBD dataset (1~XX)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run LPRVO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run LPRVO
toSave = 1;


setupParams_TUM_RGBD;


% load TUM RGBD dataset data
rawTUMRGBDdataset = rawTUMRGBDdataset_load(datasetPath, freiburg_type);


% camera calibration parameters
[TUMRGBDdataset] = getSyncTUMRGBDdataset(rawTUMRGBDdataset, imInit, M);
optsLPRVO = load_param_LPRVO;
cam = initialize_cam_TUM_RGBD(TUMRGBDdataset, optsLPRVO.imagePyramidLevel);


%% load ground truth data


% ground truth trajectory in TUM RGBD dataset
R_gc_true = zeros(3,3,M);
p_gc_true = zeros(3,M);
T_gc_true = cell(1,M);
for k = 1:M
    % camera body frame
    R_gc_true(:,:,k) = q2r(TUMRGBDdataset.vicon.q_gc_Sync(:,k));
    p_gc_true(:,k) = TUMRGBDdataset.vicon.p_gc_Sync(:,k);
    T_gc_true{k} = [ R_gc_true(:,:,k), p_gc_true(:,k);
        zeros(1,3),           1; ];
end
if (toVisualize)
    figure; hold on; axis equal;
    L = 0.1; % coordinate axis length
    A = [0 0 0 1; L 0 0 1; 0 0 0 1; 0 L 0 1; 0 0 0 1; 0 0 L 1]';
    
    for k = 1:10:M
        T = T_gc_true{k};
        B = T * A;
        plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
        plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
        plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
    end
    plot3(p_gc_true(1,:),p_gc_true(2,:),p_gc_true(3,:),'k','LineWidth',2);
    
    title('ground truth trajectory of cam0 frame')
    xlabel('x'); ylabel('y'); zlabel('z');
end


% generate ground truth trajectory in vector form
stateTrue = zeros(6,M);
stateTrue(1:3,:) = p_gc_true;
for k = 1:size(p_gc_true,2)
    [yaw, pitch, roll] = dcm2angle(R_gc_true(:,:,k));
    stateTrue(4:6,k) = [roll; pitch; yaw];
end


%% main LPRVO part


% 1. Manhattan frame tracking for LPRVO
systemInited_LPRVO = false;

R_gc1 = R_gc_true(:,:,1);
R_gc_LPRVO = zeros(3,3,M);
R_gc_LPRVO(:,:,1) = R_gc1;


% 2. make figures to visualize current status
if (toVisualize)
    % create figure
    h = figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[300 50 1400 1050]);
    ha1 = axes('Position',[0.05,0.55 , 0.4,0.4]);
    axis off;
    ha2 = axes('Position',[0.55,0.55 , 0.4,0.4]);
    axis off;
    ha3 = axes('Position',[0.05,0.05 , 0.4,0.4]);
    axis off;
    ha4 = axes('Position',[0.55,0.05 , 0.4,0.4]);
    grid on; hold on;
end


% 3. record vpInfo variables
vpInfo_LPRVO = cell(1,M);
pNV_LPRVO = cell(1,M);


% do LPRVO
for imgIdx = 1:M
    %% 1. Manhattan frame tracking
    
    % image
    imageCurForLine = getImgInTUMRGBDdataset(datasetPath, TUMRGBDdataset, cam, imgIdx, 'gray');
    imageCurForMW = getImgInTUMRGBDdataset(datasetPath, TUMRGBDdataset, cam, imgIdx, 'rgb');
    depthCurForMW = getImgInTUMRGBDdataset(datasetPath, TUMRGBDdataset, cam, imgIdx, 'depth');
    [imageCurForMW, depthCurForMW] = getImgPyramid(imageCurForMW, depthCurForMW, optsLPRVO.imagePyramidLevel);
    
    
    % for the first time in this loop
    if (~systemInited_LPRVO)
        
        % initialize and seek the dominant MF
        [R_cM, vpInfo, pNV, sNV, sPP] = seekManhattanWorld(imageCurForLine, imageCurForMW, depthCurForMW, cam, optsLPRVO);
        R_c1M = R_cM;
        R_gM = R_gc1 * R_c1M;
        systemInited_LPRVO = true;
        
        % initialize Kalman filter
        [state] = initializeVPs(R_c1M, optsLPRVO);
        
    elseif (systemInited_LPRVO)
        
        % propagation step in KF
        [state] = predictVPs(state, optsLPRVO);
        
        % track Manhattan frame
        [R_cM, vpInfo, pNV, sNV, sPP] = trackManhattanWorld(R_cM, pNV, imageCurForLine, imageCurForMW, depthCurForMW, cam, optsLPRVO);
        vpInfo_LPRVO{imgIdx} = vpInfo;
        pNV_LPRVO{imgIdx} = pNV;
        
        % correction step in KF
        [state] = updateVPs(state, R_cM, optsLPRVO);
        [state, R_cM] = extractVPs(state);
        
        
        % update current camera pose
        R_gc_current = R_gM * inv(R_cM);
        R_gc_LPRVO(:,:,imgIdx) = R_gc_current;
    end
    
    
    %% 2. update 6 DoF camera pose and visualization
    
    if (imgIdx >= 2)
        % visualize current status
        plots_status_with_true_Ronly;
    end
    
    
end

% convert camera pose representation
stateEsti_LPRVO = zeros(3,M);
for k = 1:M
    [yaw, pitch, roll] = dcm2angle(R_gc_LPRVO(:,:,k));
    stateEsti_LPRVO(:,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)


% 1) LPRVO rotation estimation trajectory
figure;
subplot(3,1,1);
plot(stateTrue(4,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPRVO(1,:),'r','LineWidth',2); hold off; axis tight; ylabel('roll (rad)');
legend('True','LPRVO Matlab');
subplot(3,1,2);
plot(stateTrue(5,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPRVO(2,:),'r','LineWidth',2); hold off; axis tight; ylabel('pitch (rad)');
subplot(3,1,3);
plot(stateTrue(6,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPRVO(3,:),'r','LineWidth',2); hold off; axis tight; ylabel('yaw (rad)');


% 2) calculate rotation matrix difference
[RMD_MEAN_LPRVO, RMD_LPRVO] = calcRMD(R_gc_LPRVO, R_gc_true);
fprintf('MEAN of RMD [deg] : %f \n' , RMD_MEAN_LPRVO);
fprintf('std. of RMD [deg] : %f \n' , std(RMD_LPRVO));


% 3) draw figures for rotation matrix difference
figure;
plot(RMD_LPRVO, 'r*'); hold on; grid on;
curve_x = 1:M;
p = polyfit(curve_x, RMD_LPRVO, 3);
curve_y = polyval(p, curve_x); curve_y(curve_y < 0) = 0;
plot(curve_x, curve_y, 'g-', 'LineWidth',5); hold off; axis tight; ylabel('RMD (deg)');
legend('RMD', 'Fitting Curve','Location','northwest');

figure;
ploterrhist(RMD_LPRVO, 'bins', 25);


%% save the experiment data for CVPR 2018

if (toSave)
    save([SaveDir '/LPRVO.mat']);
end

