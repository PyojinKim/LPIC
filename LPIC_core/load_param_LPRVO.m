function [optsLPRVO] = load_param_LPRVO
% Project:   Line And Plane Odometry
% Function: load_param_LPRVO
%
% Description:
%   get the initial parameters with respect to the algorithm
%
% Example:
%   OUTPUT:
%   optsLAPO: options for LAPO process like below
%
%   INPUT:
%
%
% NOTE:
%     The parameters below are initialized as the CVPR paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-10-25: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% line detection and matching parameters
optsLPRVO.lineDetector = 'lsd';     % 'gpa'
optsLPRVO.lineDescriptor = 'lbd';
optsLPRVO.lineLength = 60;

optsLPRVO.lineInlierThreshold = 0.8;
optsLPRVO.angleWeight = 0.7;
optsLPRVO.lengthWeight = 0.3;
optsLPRVO.minLineNum = 3;


% plane detection and tracking parameters
optsLPRVO.imagePyramidLevel = 2;
optsLPRVO.minimumDepth = 0.4;
optsLPRVO.maximumDepth = 8;
optsLPRVO.planeInlierThreshold = 0.02;
optsLPRVO.cellsize = 10;
optsLPRVO.minSampleRatio = 0.15;

optsLPRVO.numInitialization = 200;
optsLPRVO.iterNum = 200;
optsLPRVO.convergeAngle = deg2rad(0.001);
optsLPRVO.halfApexAngle = deg2rad(10);
optsLPRVO.c = 20;
optsLPRVO.ratio = 0.1;


% translation RANSAC parameters
optsLPRVO.translationInlierThreshold = 0.003;


% Kalman filter parameters
optsLPRVO.initialVPAngleNoise = deg2rad(1);
optsLPRVO.processNoise = deg2rad(0.5);
optsLPRVO.measurementNoise = deg2rad(0.5);


end






