

% Plot results 
GTPOS = zeros (3,n-(0.6*n));
VOPOS = zeros (3,n-(0.6*n));
NNPOS = zeros (3,n-(0.6*n));
VOPOS(:, 1) = DeltaVOMag(:, 1);
NNPOS(:,1)= DeltaNNMag(:,1); 
GTPOS(:,1)= DeltaGT(:,1); 
%Calculate global pose from increments 
for i = 2 : n-(0.6*n)
   
    GTPOS (:,i) = GTPOS (:,i-1) + DeltaGT (:,i); 
end 
for i = 2 : n-(0.6*n)
   
    NNPOS (:,i) = NNPOS (:,i-1) +  DeltaNNMag(:,i); 
end 
for i = 2 : n-(0.6*n)
   
    VOPOS (:,i) = VOPOS (:,i-1) +  DeltaVOMag(:,i); 
end 
%plot global pose 
len2=n-(0.6*n);
figure; hold; 
plot(GTPOS(1,1:len2),GTPOS(3,1:len2), 'g', 'LineWidth', 1.5);
plot(NNPOS(1,1:len2),NNPOS(3,1:len2), 'b', 'LineWidth', 1.5);
plot(VOPOS(1,1:len2),VOPOS(3,1:len2),'r', 'LineWidth', 1.5);
xlabel('x [m]','Fontsize',14, 'interpreter', 'latex');
ylabel('y [m]','Fontsize',14, 'interpreter', 'latex');
legend('DeltaGT', 'DeltaVOMag with NN', 'DeltaVOMag without NN', 'Fontsize', 10, 'Location', 'Best', 'interpreter', 'latex');
grid on;
grid minor;
axis equal
errorAngV = zeros(3,n-(0.6*n));
errorAngNN = zeros(3,n-(0.6*n));
error_ang_plotVO=zeros(1,n-(0.6*n));
error_ang_plotNN=zeros(1,n-(0.6*n));
dist=0;
errorvo=0;
errorNN=0;
errorNNa=0;
errorvoa=0;
%Caluclate Pose RMSE (ROOT MEAN SQUARED ERROR) 
for i=1:n-(0.6*n)
    errorvo=((GTPOS(1,i)-VOPOS(1,i))^2+(GTPOS(2,i)-VOPOS(2,i))^2+(GTPOS(3,i)-VOPOS(3,i))^2)+errorvo;
    errorNN=((GTPOS(1,i)-NNPOS(1,i))^2+(GTPOS(2,i)-NNPOS(2,i))^2+(GTPOS(3,i)-NNPOS(3,i))^2)+errorNN;
end
RMSEVOT=sqrt(errorvo/(n-(0.6*n)));
RMSENNT=sqrt(errorNN/(n-(0.6*n)));
evaluatevo=zeros(1,n-(0.6*n));
evaluatenn=zeros(1,n-(0.6*n));
%Calculate error in pose at every time step 
for i=1:n-(0.6*n)
    errorvoa=sqrt((GTPOS(1,i)-VOPOS(1,i))^2+(GTPOS(2,i)-VOPOS(2,i))^2+(GTPOS(3,i)-VOPOS(3,i))^2)+errorvoa;
    evaluatevo(i)=sqrt((GTPOS(1,i)-VOPOS(1,i))^2+(GTPOS(2,i)-VOPOS(2,i))^2+(GTPOS(3,i)-VOPOS(3,i))^2);
    errorNNa=sqrt((GTPOS(1,i)-NNPOS(1,i))^2+(GTPOS(2,i)-NNPOS(2,i))^2+(GTPOS(3,i)-NNPOS(3,i))^2)+errorNNa;
    evaluatenn(i)=sqrt((GTPOS(1,i)-NNPOS(1,i))^2+(GTPOS(2,i)-NNPOS(2,i))^2+(GTPOS(3,i)-NNPOS(3,i))^2);
end
%calculate error in ang at every time step 
for i=1:n-(0.6*n)
    roterrorAng (:,:,i)= (Minc(:,1:3,i)\(Mvo(:,1:3,i)));
    errorAngV(:,i)=(rotm2eul(roterrorAng (:,:,i)))';
    error_ang_plotVO(i)=norm(errorAngV(:,i))*180/pi;
end
for i=1:n-(0.6*n)
  roterrorAng2(:,:,i)= (Minc(:,1:3,i)\(rotNN(:,:,i)));
  errorAngNN(:,i)=(rotm2eul(roterrorAng2 (:,:,i)))';
  error_ang_plotNN(i)=norm(errorAngNN(:,i))*180/pi;
end
%plot translation error vs orientation error 
figure;
subplot(2, 1, 1); hold;
plot(evaluatenn, 'b', 'LineWidth', 1.5);
plot(evaluatevo, 'r', 'LineWidth', 1.5);
title('Translation Error')
xlabel('Frame Number','Fontsize',14, 'interpreter', 'latex');
ylabel('Error [m]','Fontsize',14, 'interpreter', 'latex');
legend('DeltaVOMag with NN', 'DeltaVOMag without NN', 'Fontsize', 10, 'Location', 'Best', 'interpreter', 'latex');
grid on;
grid minor;
subplot(2, 1, 2); hold;
plot(error_ang_plotNN(1,:), 'b', 'LineWidth', 1.5);
plot(error_ang_plotVO(1,:), 'r', 'LineWidth', 1.5);
title('Incremental Orientation Error')
xlabel('Frame Number','Fontsize',14, 'interpreter', 'latex');
ylabel('Error [deg]','Fontsize',14, 'interpreter', 'latex');
legend('DeltaVOMag with NN', 'DeltaVOMag without NN', 'Fontsize', 10, 'Location', 'Best', 'interpreter', 'latex');
grid on;
grid minor;
%calculate testing sequence distance 
for i=2:n-(0.6*n)
   dist=dist+norm(GTPOS(:,i)-GTPOS(:,i-1));
end
errorangvo=0;
errorangNN=0;
rmseangvo=0;
rmseangNN=0;
   for i=1:n-(0.6*n)
         errorangvo=norm(rotm2eul(eul2rotm(result_posRGGT(:,i)')\eul2rotm(result_posRGVO(:,i)'))'*180/pi)^2+errorangvo;
         errorangNN=norm(rotm2eul(eul2rotm(result_posRGGT(:,i)')\eul2rotm(result_posRGNN(:,i)'))'*180/pi)^2+errorangNN;
    end
    rmseangvo=sqrt(errorangvo/(n-(0.6*n)));
     rmseangNN=sqrt(errorangNN/(n-(0.6*n)));
  


     
     
     
         