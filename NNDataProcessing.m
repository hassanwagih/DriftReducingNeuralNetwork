NNoutput= net17(NNinput);
for i=1:n
quatAng(:,i)=calculateExactQuatExp(NNoutput(:,i));
rotNN(:,:,i)=quat2rotm(quatAng(:,i)');
quatAng(:,i)=0;
end 
RposNN = eye(3);
posNN = [0;0;0];
%calulating delta pose using angles estimated by the NN for the remaining 40% 

for i=1:n-(0.6*n)
Radj(:,:,i) = rotNN(:,:,i+((0.6*n)-1));
    tradj(:,i) = Mvo(:,4,i+((0.6*n)-1));
    
    %% Estimated pose relative to global frame at t = n*0.6
    posNN = posNN + RposNN * tradj(:,i);
    
    RposNN = Radj(:,:,i) * RposNN;
    

    
    
    result_posTGNN(:,i) = tradj(:,i);
    result_posRGNN(:,:,i) = Radj(:,:,i);
    
    NN_Gorint(:,i)=rotm2eul(RposNN)';
    NN_Gpos(:,i) = posNN;
    
end 
NN_DeltaVar= NN_Gpos; 
%NN_DeltaVar(1,1:n)=-NN_Gpos(1,1:n);
for j=2:n-(0.6*n)
    
    deltascaleNN(1,j)=NN_DeltaVar(1,j)-(NN_DeltaVar(1,j-1));
    deltascaleNN(2,j)=NN_DeltaVar(2,j)-(NN_DeltaVar(2,j-1));
    deltascaleNN(3,j)=NN_DeltaVar(3,j)-(NN_DeltaVar(3,j-1)); 
    magNN(j)=norm(delta(:,j+0.6*n-1))/norm(deltascaleNN(:,j)); %calculate magnitude of scale 

end 
for j=2:n-(0.6*n)
    if magNN(j)==inf
        
        magNN(j)=0; 
    end 
end
%use magnitude to adjust Monocular Scale 
DeltaNNMag=zeros(3,n-(0.6*n));
for jjj=1:n-(0.6*n)
    if jjj>1
    DeltaNNMag(1,jjj)=(NN_DeltaVar(1,jjj)-NN_DeltaVar(1,jjj-1))*magNN(jjj);
    DeltaNNMag(2,jjj)=(NN_DeltaVar(2,jjj)-NN_DeltaVar(2,jjj-1))*magNN(jjj);
    DeltaNNMag(3,jjj)=(NN_DeltaVar(3,jjj)-NN_DeltaVar(3,jjj-1))*magNN(jjj);
    
    end
end
RposVO = eye(3);
posVO = [0;0;0];
for i=1:n-(0.6*n)
Radj2(:,:,i) = Mvo(1:3,1:3,i+((0.6*n)-1));
    tradj2(:,i) = Mvo(:,4,i+((0.6*n)-1));
    
    %% Estimated pose relative to global frame at t = n*0.6
    posVO = posVO + RposVO * tradj2(:,i);
    
    RposVO = Radj2(:,:,i) * RposVO;
    

    result_posTGVO(:,i) = tradj2(:,i);
    result_posRGVO(:,:,i) = Radj2(:,:,i);
    VO_Gorint(:,i)=rotm2eul(RposVO)';
    VO_Gpos(:,i) = posVO;
    
end 

VO_DeltaVar= VO_Gpos; 

for j=2:n-(0.6*n)
    
    deltascaleVO(1,j)=VO_DeltaVar(1,j)-(VO_DeltaVar(1,j-1));
    deltascaleVO(2,j)=VO_DeltaVar(2,j)-(VO_DeltaVar(2,j-1));
    deltascaleVO(3,j)=VO_DeltaVar(3,j)-(VO_DeltaVar(3,j-1)); 
    magVO(j)=norm(delta(:,j+0.6*n-1))/norm(deltascaleVO(:,j)); 

end 
for j=2:n-(0.6*n)
    if magVO(j)==inf
        
        magVO(j)=0; 
    end 
end
DeltaVOMag=zeros(3,n-(0.6*n));
for jjj=1:n-(0.6*n)
    if jjj>1
    DeltaVOMag(1,jjj)=(VO_DeltaVar(1,jjj)-VO_DeltaVar(1,jjj-1))*magVO(jjj);
    DeltaVOMag(2,jjj)=(VO_DeltaVar(2,jjj)-VO_DeltaVar(2,jjj-1))*magVO(jjj);
    DeltaVOMag(3,jjj)=(VO_DeltaVar(3,jjj)-VO_DeltaVar(3,jjj-1))*magVO(jjj);
    
    end
end
RposGT = eye(3);
posGT = [0;0;0];
for i=1:n-(0.6*n)
Radj3(:,:,i) = Minc(1:3,1:3,i+((0.6*n)-1));
    tradj3(:,i) = Minc(:,4,i+((0.6*n)-1));
    
    %% Estimated pose relative to global frame at t = n*0.6
    posGT = posGT + RposGT * tradj3(:,i);
    
    RposGT = Radj3(:,:,i) * RposGT;
    

    result_posTGGT(:,i) = tradj3(:,i);
    result_posRGGT(:,:,i) = Radj3(:,:,i);
    GT_Gorint(:,i)=rotm2eul(RposGT)';
    GT_Gpos(:,i) = posGT;
    
end 
GT_DeltaVar= GT_Gpos; 
%GT_DeltaVar(1,1:n-(0.6*n))=-GT_Gpos(1,1:n-(0.6*n));
for j=2:n-(0.6*n)
    
    deltascaleGT(1,j)=GT_DeltaVar(1,j)-(GT_DeltaVar(1,j-1));
    deltascaleGT(2,j)=GT_DeltaVar(2,j)-(GT_DeltaVar(2,j-1));
    deltascaleGT(3,j)=GT_DeltaVar(3,j)-(GT_DeltaVar(3,j-1)); 
    mag24(j)=norm(delta(:,j+0.6*n-1))/norm(deltascaleGT(:,j)); 

end 
for j=2:n-(0.6*n)
    if mag24(j)==inf
        
        mag24(j)=0; 
    end 
end
DeltaGT=zeros(3,n-(0.6*n));
for jjj=1:n-(0.6*n)
    if jjj>1
    DeltaGT(1,jjj)=(GT_DeltaVar(1,jjj)-GT_DeltaVar(1,jjj-1));
    DeltaGT(2,jjj)=(GT_DeltaVar(2,jjj)-GT_DeltaVar(2,jjj-1));
    DeltaGT(3,jjj)=(GT_DeltaVar(3,jjj)-GT_DeltaVar(3,jjj-1));
    
    end
end

 
