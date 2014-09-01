function [targets] = computeTargetPos_co_dex()
%compute target positions (numtargets x 2)
targets = load('co_targetpos_dex.dat');
targets = targets / 100;

origin = [0 0.02];
scale = 0.08;
targets = (targets - repmat(origin,9,1)) / scale;




