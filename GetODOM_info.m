function [ROrint,RLoc,D1,D2,inliersT1,inliersT2] = GetODOM_info(I_1, I_2, cameraParamsl, PLOT_CONTORL)
[ inliersT1,inliersT2,D1,D2] = GetFeatures(I_1, I_2, PLOT_CONTORL);
[orientation, location, inlierIdx] = GetPose(inliersT1,inliersT2, cameraParamsl);
ROrint = orientation;
RLoc = location;
end
