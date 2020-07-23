%% TUM RGBD dataset can be downloaded from: http://vision.in.tum.de/data/datasets/rgbd-dataset


switch( expCase )
    
    case 1 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg2_rpy';
        freiburg_type = 'fr2';
        
        imInit      = 1;      % first image index, (1-based index)
        M           = 3221;  % number of images
        
        
    case 2 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg2_desk';
        freiburg_type = 'fr2';
        
        imInit      = 1;       % first image index, (1-based index)
        M           = 2893;  % number of images
        
        
    case 3 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_long_office_household';
        freiburg_type = 'fr3';
        
        imInit      = 1;        % first image index, (1-based index)
        M           = 2488;   % number of images
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%% Category: Structure vs. Texture  %%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    case 4 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_nostructure_notexture_near_withloop';
        freiburg_type = 'fr3';
        
        imInit      = 451;   % first image index, (1-based index)
        M           = 149;   % number of images
        
        
    case 5 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_nostructure_texture_far';
        freiburg_type = 'fr3';
        
        imInit      = 1;     % first image index, (1-based index)
        M           = 351;  % number of images
        
        
    case 6 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_nostructure_texture_near_withloop';
        freiburg_type = 'fr3';
        
        imInit      = 1;     % first image index, (1-based index)
        M           = 1639; % number of images
        
        
    case 7 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_structure_notexture_far';
        freiburg_type = 'fr3';
        
        imInit      = 1;     % first image index, (1-based index)
        M           = 794;  % number of images
        
        
    case 8 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_structure_notexture_near';
        freiburg_type = 'fr3';
        
        imInit      = 1;     % first image index, (1-based index)
        M           = 1054; % number of images
        
        
    case 9 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_structure_texture_far';
        freiburg_type = 'fr3';
        
        imInit      = 1;     % first image index, (1-based index)
        M           = 907;  % number of images
        
        
    case 10 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_structure_texture_near';
        freiburg_type = 'fr3';
        
        imInit      = 1;      % first image index, (1-based index)
        M           = 1057;  % number of images
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%% Category: 3D Object Reconstruction  %%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    case 11 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_cabinet';
        freiburg_type = 'fr3';
        
        imInit      = 1;      % first image index, (1-based index)
        M           = 1112; % number of images
        
        
    case 12 % tuning is done
        datasetPath = 'G:/TUMRGBDdataset/rgbd_dataset_freiburg3_large_cabinet';
        freiburg_type = 'fr3';
        
        imInit      = 451;  % first image index, (1-based index)
        M           = 530;  % number of images
        
        
end