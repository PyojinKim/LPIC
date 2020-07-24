function plot_display_text_with_true_Ronly(R_cM, sNV, vpInfo, R_gc_LPIC, R_gc_true, imgIdx, optsLPIC)


% number of sNV
numNV = computeNumInXYZCone(R_cM, sNV, optsLPIC.halfApexAngle);


% number of lines
numLines = zeros(1,3);
numLines(1) = vpInfo(1).n;
numLines(2) = vpInfo(2).n;
numLines(3) = vpInfo(3).n;


% rotation matrix difference with ground-truth
R_gc2_true = R_gc_true(:,:,imgIdx);
R_gc2_LAPO = R_gc_LPIC(:,:,imgIdx);
RMD = acos((trace(R_gc2_true.' * R_gc2_LAPO)-1)/2) * (180/pi);


% rotation variation
R_21_LPRVO = inv(R_gc_LPIC(:,:,imgIdx)) * R_gc_LPIC(:,:,imgIdx-1);
R_12_LPRVO = inv(R_21_LPRVO);
RV = acos((trace(R_12_LPRVO)-1)/2) * (180/pi);


% display current status
text(7, 10, sprintf('sNV: %05d, %05d, %05d', numNV(1), numNV(2), numNV(3)), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 22, sprintf('lines: %05d, %05d, %05d', numLines(1), numLines(2), numLines(3)), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 34, sprintf('RMD (deg): %f', RMD), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(7, 46, sprintf('RV (deg): %f', RV), 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');


end

