function [ inliersT1,inliersT2,descriptors_1,descriptors_2] = ...
    GetFeatures(I_1, I_2, PLOT_CONTORL)
 T1Features = detectSURFFeatures(I_1, 'MetricThreshold', 1000);%Use SURF Features detector on image at Time T
 T2Features = detectSURFFeatures(I_2, 'MetricThreshold', 1000);%Use SURF Features detector on image at Time T+1
[frame_T1,Points_T1] = extractFeatures(I_1,T1Features,'Upright', true);
[frame_T2,Points_T2] = extractFeatures(I_2,T2Features,'Upright', true);
indexPairs = matchFeatures(frame_T1,frame_T2,'Unique', true) ; % Match features using forward_backward Matching
inliersT1= Points_T1(indexPairs(:,1));
inliersT2 = Points_T2(indexPairs(:,2));
descriptors_1 = frame_T1(indexPairs(:,1),:);
descriptors_2 = frame_T2(indexPairs(:,2),:);

if PLOT_CONTORL == 1
% figure;
% hold on;
% J = insertMarker(I_1,inliersT1,'o','size',7,'color','red');
% imshow(J)
% title('Features Left Image at time T');
% hold off;
% 
% figure;
% hold on;
% G = insertMarker(I_2,inliersT2,'size',7);
% imshow(G)
% title('Features Left Image at time Tplus1');
% hold off;

showMatchedFeatures(I_1, I_2, inliersT1, inliersT2);
title('matched features in left images');
legend('t','tplus1')
hold off;
end



end
