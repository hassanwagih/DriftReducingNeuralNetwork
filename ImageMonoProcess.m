clear all
%retrive data from data set 

seqnames = {'00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '10'};

% for i = 1 : 11
i = 06;
    
seqname = seqnames{i};

if strcmp(seqname, '00')
    n = 4540;
elseif strcmp(seqname, '01')
    n = 1100;
elseif strcmp(seqname, '02')
    n = 4660;
elseif strcmp(seqname, '03')
    n = 800;
elseif strcmp(seqname, '04')
    n = 270;
elseif strcmp(seqname, '05')
    n = 2760;
elseif strcmp(seqname, '06')
    n = 1100;
elseif strcmp(seqname, '07')
    n = 1100;
elseif strcmp(seqname, '08')
    n= 4070;
elseif strcmp(seqname, '09')
    n = 1590;
elseif strcmp(seqname, '10')
    n = 1200;
end

direct0Name = strcat('.\data_odometry_gray\dataset\sequences\', seqname, '\image_0\*.png');
image0name = dir(direct0Name);
direct1Name =  strcat('.\data_odometry_gray\dataset\sequences\', seqname, '\image_1\*.png');
image1name = dir(direct1Name); 
calibname = strcat('.\data_odometry_calib\dataset\sequences\', seqname, '\calib.txt');
TableArray = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
A = table2array(TableArray);
Pl = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
K1 = Pl(1:3,1:3);
PLOT_CONTORL = 0;%Plot control value.
Rpos = eye(3);%initialize rotation matrix 
pos = [0;0;0];%initialize translation vector 
NN_training_array=zeros(11,n); %Define initial training array 
Mvo=zeros(3,4,n); %Define Visual Odometery increments 
if PLOT_CONTORL
    figure;
    hold on;
end
for i = 1:n
   Left_image1 = strcat('.\data_odometry_gray\dataset\sequences\', seqname, '\image_0\', image0name(i).name);
   I1_l = imread(Left_image1);%image at time T
    file_name01 = strcat('.\data_odometry_gray\dataset\sequences\', seqname, '\image_0\', image0name(i+1).name);
    I2_l = imread(file_name01);%image at time T+1 
    dims = size(I2_l);%image size 
    cam1 = cameraIntrinsics([K1(1, 1), K1(2,2)], [K1(1, 3), K1(2, 3)], dims); % get camera intrinstics form the KITTI file 
    [ROrint,RLoc,D1,D2,I1,I2] = GetODOM_info(I1_l, I2_l, cam1, PLOT_CONTORL); % get vehicle odometry from both images at time T and T+1
    s=size(D1);
    S=s(1)*s(2);
    s2=size(D2);
    S2=s2(1)*s2(2);
    training2=I1.Location';
    training3=I2.Location';
    training4 = training3 - training2 ; 
    train7 = training4(1,[1:50]);
    train8 = training4(2,[1:50]);
   training4=training4(:,[1:50]);
   training4= reshape(training4,[],1);
   %Acquire statistical moments for the features movments 
   NN_training_array(1,i)= mean(train7);
   NN_training_array(2,i)=rms(train7);
   NN_training_array(3,i)=var(train7);
   NN_training_array(4,i)=skewness(train7);
   NN_training_array(5,i)=mean(train8);
   NN_training_array(6,i)=rms(train8);
   NN_training_array(7,i)=var(train8);
   NN_training_array(8,i)=skewness(train8);
   % calculating delta step for camera 
    R = ROrint;
    tr = RLoc';
    tr2=[1,0,0;0,1,0;0,0,1]*tr;
    R2= rotm2eul(R); 
    R2=R2';
    %perform frame rotation due as per KITTI sensor setup.
    R2=[-1,0,0;0,-1,0;0,0,1]*R2;
    R2=eul2rotm(R2');
    Mvo(:,:,i)=[R2,tr2];
   
end

%% ground truth increment calculations
 posesname = strcat('.\data_odometry_poses\dataset\poses\', seqname, '.txt');
 TableArray = readtable(posesname,'Delimiter','space','ReadRowNames',false,'ReadVariableNames',false);
 A = table2array(TableArray);
 M = zeros(3,4,length(A));
 
 for i = 1: length(A)
     M(1:3,1:4,i) = [A(i,1:4);A(i,5:8);A(i,9:12)];
 end
 %Calculate ground truth increments 
 Minc=zeros(size(M,1),size(M,2),size(M,3)-1);
for i=1:length(A)-1
    M1= [M(:,:,i);0,0,0,1];
    M2= [M(:,:,i+1);0,0,0,1];
    deltaT=M1\M2;
    deltaT=deltaT(1:3,:);
    Minc(:,:,i)=deltaT;
end
len = length(M);
Po = zeros(3,len);
for j = 1 : len
    Po(:,j) = M(:,:,j)*[0;0;0;1];
end

pos = [0;0;0];
Rpos = eye(3);
for k = 1:len
    pos_temp = Po(:,k);
    Rpos_temp = M(:,1:3,k);
    %tr(:,k) = inv(Rpos)*(pos_temp - pos);
    R(:,:,k) = inv(Rpos)*Rpos_temp;
    pos = pos_temp;
    Rpos = Rpos_temp;
end

delta=zeros(3,n);

 for kk=1:n
    if kk>1
    delta(1,kk)=Po(1,kk)-Po(1,kk-1);
    delta(2,kk)=Po(2,kk)-Po(2,kk-1);
    delta(3,kk)=Po(3,kk)-Po(3,kk-1);
    end
 end

NNinput=zeros(11,n);
NNinput=NN_training_array(1:8,:);
AngdevV=zeros(3,n); 
AngdevGT=zeros(3,n);
%change angles to tanget space 
for i=1:n 
Quatv=fromRotToQuat(Mvo(:,1:3,i)); 
AngdevV(:,i)=calculateExactQuatLog(Quatv);
Quatv2=fromRotToQuat(Minc(:,1:3,i)); 
AngdevGT(:,i)=calculateExactQuatLog(Quatv2);
end 
% 60% of the set used as training 
NNinput(9:11,:)=AngdevV;% delta angles in tangent space of the camera 
seq00T=NNinput(:,1:n*0.6);
seq00L=AngdevGT(:,1:n*0.6);% delta angles in tangent space of the ground truth (label) 

save(seqname, 'seq00T', 'seq00L');

% end
    