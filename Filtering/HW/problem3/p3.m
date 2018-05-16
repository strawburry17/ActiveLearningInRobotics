clear;
close all;
clc;

distortion = linspace(0.70,1.3,11);

sim_err = zeros(length(distortion),100);

for i = 1:length(distortion)
    for j = 1:100
        sim_err(i,j) = simKalman(distortion(i));
    end
    avg_err(i) = mean(sim_err(i,:));
    fprintf("distortion: %f, average error = %f\n",distortion(i),avg_err(i));
    hold on
end

bar(distortion,avg_err)