%% fourierTraj.m
function ck = fourierTraj(xvec,tvec,T,k,L1,L2)

FKvec = zeros(1,length(xvec));
for i = 1:length(xvec)
    Fkvec(i) = fourierBasisPoint([xvec(1,i),xvec(2,i)],k,L1,L2);
end

ck = (1/T)*trapz(tvec,Fkvec);

end