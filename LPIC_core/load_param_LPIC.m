function [optsLPIC] = load_param_LPIC
% Project:   Line And Plane Odometry
% Function: load_param_LPIC
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
optsLPIC.lineDetector = 'lsd';     % 'gpa'
optsLPIC.lineDescriptor = 'lbd';
optsLPIC.lineLength = 60;

optsLPIC.lineInlierThreshold = 0.8;
optsLPIC.angleWeight = 0.7;
optsLPIC.lengthWeight = 0.3;
optsLPIC.minLineNum = 3;


% plane detection and tracking parameters
optsLPIC.imagePyramidLevel = 2;
optsLPIC.minimumDepth = 0.4;
optsLPIC.maximumDepth = 8;
optsLPIC.planeInlierThreshold = 0.02;
optsLPIC.cellsize = 10;
optsLPIC.minSampleRatio = 0.15;

optsLPIC.numInitialization = 200;
optsLPIC.iterNum = 200;
optsLPIC.convergeAngle = deg2rad(0.001);
optsLPIC.halfApexAngle = deg2rad(10);
optsLPIC.c = 20;
optsLPIC.ratio = 0.1;


% translation RANSAC parameters
optsLPIC.translationInlierThreshold = 0.003;


% Kalman filter parameters
optsLPIC.initialVPAngleNoise = deg2rad(1);
optsLPIC.processNoise = deg2rad(0.5);
optsLPIC.measurementNoise = deg2rad(0.5);


end






