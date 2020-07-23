function [rawTUMRGBDdataset] = rawTUMRGBDdataset_load(datasetPath, freiburg_type)
% Project:    Depth enhanced visual odometry
% Function:  rawTUMRGBDdataset_load
%
% Description:
%   get raw rgb, depth, and Vicon data from TUM RGBD dataset
%
% Example:
%   OUTPUT:
%   rawTUMRGBDdataset:
%
%   INPUT:
%   datasetPath: directory of folder which includes the groundtruth.txt from this m.file
%   freiburg_type: type of freiburg
%
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-12-10: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


%% import data from text files


% import ground-truth data
delimiter = ' ';
headerlinesIn = 3;
textData_groundtruth = importdata([datasetPath '/groundtruth.txt'], delimiter, headerlinesIn);


% import time_sync (rgb and depth) data
fileID = fopen([datasetPath, '/time_sync.txt'], 'r');
textData_timesync = textscan(fileID, '%f64 %s %f64 %s');
fclose(fileID);


%% load ground-truth and rgb data


% load ground-truth data
rawTUMRGBDdataset.vicon.time = textData_groundtruth.data(:,1).';
rawTUMRGBDdataset.vicon.p_gc = [textData_groundtruth.data(:,2).'; textData_groundtruth.data(:,3).'; textData_groundtruth.data(:,4).'];
rawTUMRGBDdataset.vicon.q_gc = [textData_groundtruth.data(:,8).'; textData_groundtruth.data(:,5).'; textData_groundtruth.data(:,6).'; textData_groundtruth.data(:,7).'];


% load synchronized rgb data
switch freiburg_type
    case 'default'
        K = eye(3);
        K(1,1) = 525.0;  % fx
        K(2,2) = 525.0;  % fy
        K(1,3) = 319.5;  % cx
        K(2,3) = 239.5;  % cy
        distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4
        
    case 'fr1'
        K = eye(3);
        K(1,1) = 525.0;  % fx
        K(2,2) = 525.0;  % fy
        K(1,3) = 319.5;  % cx
        K(2,3) = 239.5;  % cy
        distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4
        
    case 'fr2'
        K = eye(3);
        K(1,1) = 525.0;  % fx
        K(2,2) = 525.0;  % fy
        K(1,3) = 319.5;  % cx
        K(2,3) = 239.5;  % cy
        distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4
        
    case 'fr3'
        K = eye(3);
        K(1,1) = 535.4;  % fx
        K(2,2) = 539.2;  % fy
        K(1,3) = 320.1;  % cx
        K(2,3) = 247.6;  % cy
        distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4
        
end
rawTUMRGBDdataset.rgb.time = textData_timesync{1}.';
rawTUMRGBDdataset.rgb.imgName = textData_timesync{2}.';
rawTUMRGBDdataset.rgb.K = K;
rawTUMRGBDdataset.rgb.distortion = distortion;


% load synchronized depth data
rawTUMRGBDdataset.depth.time = textData_timesync{3}.';
rawTUMRGBDdataset.depth.imgName = textData_timesync{4}.';
rawTUMRGBDdataset.depth.scaleFactor = 5000;


end
