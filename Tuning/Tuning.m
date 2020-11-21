%clc
clear
close all

% Define the following values based on the subject number and block tuning
% number
blocktuning = 2; % 1 is when you find tuning by zero damping, 2 is variable damping, 3 is variable damping
subnum = 8;



for i=1:2
    for j =1:10
        str = ['D:\Adaptive\Experiment_rangecheck\Data\Subject',num2str(subnum),'\Direction',num2str(i),'\Block',num2str(2*blocktuning-2+i),'\Trial',num2str(j),'\KukaData.txt'];
        data = load(str);
        if i==1
            n = 0;
        else
            n = 1;
        end
        
        xdot = data(:,20 - n);
        xdotdot = data(:,22 - n);
        
        [kp(n + 1,j), kn(n + 1,j)] = GetKs(xdot, xdotdot);
    end
end

kpave = mean(kp,2);
knave = mean(kn,2);

disp("kpml: ");
disp(kpave(1));
disp("knml: ");
disp(knave(1));

disp("kpap: ");
disp(kpave(2));
disp("knap: ");
disp(knave(2));