function ref = footsteps(steps,nTimeSteps)
% footsteps
% given a vector of footsteps STEPS and a number of time steps 
% nTimeSteps is the number of time steps between each footstep

nTimeSteps;
chunk = ones(1,nTimeSteps);
ref = zeros(1,length(chunk)*length(steps));
for i=1:length(steps)
    ref((i-1)*nTimeSteps+1:(i-1)*nTimeSteps+nTimeSteps) = chunk*steps(i);
end

