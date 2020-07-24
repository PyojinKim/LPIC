if (toVisualize)
    %% prerequisite to visualize
    
    stateEsti = zeros(3, imgIdx);
    for k = 1:imgIdx
        [yaw, pitch, roll] = dcm2angle(R_gc_LPIC(:,:,k));
        stateEsti(:,k) = [roll; pitch; yaw];
    end
    
    %% update lines & plane on RGB image
    
    axes(ha1); cla;
    plot_plane_image(pNV, sNV, sPP, imageCurForMW, optsLPIC); hold on;
    plots_extended_lines(R_cM, vpInfo, imageCurForLine, cam, optsLPIC);
    plot_display_text_with_true_Ronly(R_cM, sNV, vpInfo, R_gc_LPIC, R_gc_true, imgIdx, optsLPIC); hold off;
    title('point, line, plane tracking image');
    
    %% update all lines on gray image
    
    axes(ha2); cla;
    plot_image_lines(imageCurForLine, cam, optsLPIC); hold off;
    title('all detected lines in the image');
    
    %% update rotational motion of camera
    
    axes(ha3); cla;
    hold on; grid on; axis equal; view(130,30);
    plot_Manhattan_camera_frame(R_cM, R_gc_LPIC(:,:,imgIdx));
    plot_true_camera_frame(R_gc_true(:,:,imgIdx)); hold off;
    title('unit sphere in SO(3)');
    
    %% update 3D trajectory of camera
    
    axes(ha4); cla;
    plot(stateEsti(2,1:imgIdx), 'm', 'LineWidth', 2); hold on; grid on;
    plot(stateTrue(5,1:imgIdx), 'k', 'LineWidth', 2); hold off; ylabel('pitch (rad)');
    title('estimated & true pitch angle in rad');
    
    %% save current figure
    
    if (toSave)
        % save directory for MAT data
        SaveDir = [datasetPath '/CVPR2018'];
        if (~exist( SaveDir, 'dir' ))
            mkdir(SaveDir);
        end
        
        % save directory for images
        SaveImDir = [SaveDir '/LPIC'];
        if (~exist( SaveImDir, 'dir' ))
            mkdir(SaveImDir);
        end
        
        pause(0.01); refresh;
        saveImg = getframe(h);
        imwrite(saveImg.cdata , [SaveImDir sprintf('/%06d.png', imgIdx)]);
    end
end
