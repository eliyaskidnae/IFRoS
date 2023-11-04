function errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% 
% Given two list of coordinates (CL1uv and CL2uv) on two images and the
% homography that relates them (H12) this function will compute an error vector
% (errorVec).  This vector will contain the Euclidean distance between each
% point in CL1uv and its corresponding point in CL2uv after applying the
% homography  H12.


% insert your code here

% remove this line after creating your function
errorVec = zeros(size(CL1uv,1),1); 
 