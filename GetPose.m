function [orientation, location, inlierIdx,I1,I2] = ...
    GetPose(inliersT1, inliersT2, cameraParamsl)

if ~isnumeric(inliersT1)
    inliersT1 = inliersT1.Location;
end

if ~isnumeric(inliersT2)
    inliersT2 = inliersT2.Location;
end

for i = 1:100
    VPF = 0;
    y = 0; 
    try
    [E, inlierIdx] = estimateEssentialMatrix(inliersT1, inliersT2,...
        cameraParamsl,'MaxNumTrials' ,1000,'Confidence',99,'MaxDistance' ,.1);
    catch
        continue;
    end
    r = sum(inlierIdx) / numel(inlierIdx); % test if the used inliers are suffient 
    if r < .32 %thershold value for inlier index which which is true compared to the total  
        continue;
    end
    y = sum(inlierIdx);
    inlierPoints1 = inliersT1(inlierIdx, :);
    inlierPoints2 = inliersT2(inlierIdx, :);    
    [orientation, location, VPF] = relativeCameraPose(E, cameraParamsl, inlierPoints1, inlierPoints2);
    
    if VPF > .8 %valid pointfraction value acceptance criteria 
       return;
    end
    
end
 orientation = eye(3); %Orientation matrix value if value of validPointFraction is still low 
 location = [0,0,0]; %Orientation matrix value if value of validPointFraction is still low 

end

