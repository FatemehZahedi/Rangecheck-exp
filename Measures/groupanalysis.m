clc
clear
close all


numofsubject = 8;

for sub=1:numofsubject
    clear AP_measures ML_measures
    str = ['C:\Users\fzahedi1\OneDrive - Arizona State University\Projects\Adaptive\Bayesian Optimization\Experiment_rangecheck\Data\Subject',num2str(sub),'\Measures.mat'];
    load(str);
    
    AllmeasuresAP(:,:,sub) = AP_measures;
    AllmeasuresML(:,:,sub) = ML_measures;
    
    ave_ML(sub,:) = mean(ML_measures(:,1:6));
    st_ML(sub,:) = std(ML_measures(:,1:6));

    ave_AP(sub,:) = mean(AP_measures(:,1:6));
    st_AP(sub,:) = std(AP_measures(:,1:6));
    
    COVall(sub,:,:) = COV;
    
end

COVave = mean(COVall);
Weight = 1/COVave; % 2 in the third dimension is AP

for j= [1, 3, 4]
    for i = 1:numofsubject
        AP(i,:,j) = AllmeasuresAP(:,j,i)';
        ML(i,:,j) = AllmeasuresML(:,j,i)';
    end
end

temp = mean(ave_ML);
tempst = std(ave_ML);
Overallave(1,:) = temp([1, 3, 4]);
Overallave(3,:) = tempst([1, 3, 4]);
temp = mean(ave_AP);
tempst = std(ave_AP);
Overallave(2,:) = temp([1, 3, 4]);
Overallave(4,:) = tempst([1, 3, 4]);

dampingAP = [-50, -40, -30, -20, -10];
dampingML = [-25, -20, -15, -10, -5];

maxOSallsubjectAP = max(max(AP(:,:,1)));
minOSallsubjectAP = min(min(AP(:,:,1)));
maxOSmeanAP = max(mean(AP(:,:,1)));
minOSmeanAP = min(mean(AP(:,:,1)));

maxSPEEDallsubjectAP = max(max(AP(:,:,3)));
minSPEEDallsubjectAP = min(min(AP(:,:,3)));
maxSPEEDmeanAP = max(mean(AP(:,:,3)));
minSPEEDmeanAP = min(mean(AP(:,:,3)));

maxFORCEallsubjectAP = max(max(AP(:,:,4)));
minFORCEallsubjectAP = min(min(AP(:,:,4)));
maxFORCEmeanAP = max(mean(AP(:,:,4)));
minFORCEmeanAP = min(mean(AP(:,:,4)));

maxOSallsubjectML = max(max(ML(:,:,1)));
minOSallsubjectML = min(min(ML(:,:,1)));
maxOSmeanML = max(mean(ML(:,:,1)));
minOSmeanML = min(mean(ML(:,:,1)));

maxSPEEDallsubjectML = max(max(ML(:,:,3)));
minSPEEDallsubjectML = min(min(ML(:,:,3)));
maxSPEEDmeanML = max(mean(ML(:,:,3)));
minSPEEDmeanML = min(mean(ML(:,:,3)));

maxFORCEallsubjectML = max(max(ML(:,:,4)));
minFORCEallsubjectML = min(min(ML(:,:,4)));
maxFORCEmeanML = max(mean(ML(:,:,4)));
minFORCEmeanML = min(mean(ML(:,:,4)));

maxmin(:,:,2) = [maxOSallsubjectAP maxSPEEDallsubjectAP maxFORCEallsubjectAP;
                 minOSallsubjectAP minSPEEDallsubjectAP minFORCEallsubjectAP;
                 maxOSmeanAP maxSPEEDmeanAP maxFORCEmeanAP;
                 minOSmeanAP minSPEEDmeanAP minFORCEmeanAP];
maxmin(:,:,1) = [maxOSallsubjectML maxSPEEDallsubjectML maxFORCEallsubjectML;
                 minOSallsubjectML minSPEEDallsubjectML minFORCEallsubjectML;
                 maxOSmeanML maxSPEEDmeanML maxFORCEmeanML;
                 minOSmeanML minSPEEDmeanML minFORCEmeanML];

figure
subplot(3,2,1)
plot(dampingML,mean(ML(:,:,1)),'LineWidth',1.5)
title('ML');
ylabel('Overshoot');
xticks([]);
subplot(3,2,2)
plot(dampingAP,mean(AP(:,:,1)),'LineWidth',1.5)
xticks([]);
title('AP');
subplot(3,2,3)
plot(dampingML,mean(ML(:,:,3)),'LineWidth',1.5)
xticks([]);
ylabel('MeanSpeed');
subplot(3,2,4)
plot(dampingAP,mean(AP(:,:,3)),'LineWidth',1.5)
xticks([]);
subplot(3,2,5)
plot(dampingML,mean(ML(:,:,4)),'LineWidth',1.5)
ylabel('MeanRMS_{force}');
xlabel('b_{LB}');
subplot(3,2,6)
plot(dampingAP,mean(AP(:,:,4)),'LineWidth',1.5)
xlabel('b_{LB}');

figure
labels = {'OS';'Speed';'Force'};
subplot(1,2,1)
for j=1:size(Overallave,2)
bar(j,Overallave(1,j))
hold on
error= Overallave(3,j);
errorbar(j,Overallave(1,j),zeros(size(error)),error , '.k');
end
title('ML')
set(gca, 'XTickLabel',labels, 'XTick',1:numel(labels))

subplot(1,2,2)
for j=1:size(Overallave,2)
bar(j,Overallave(2,j))
hold on
error= Overallave(4,j);
errorbar(j,Overallave(2,j),zeros(size(error)),error , '.k');
end
title('AP')
set(gca, 'XTickLabel',labels, 'XTick',1:numel(labels))

save(['C:\Users\fzahedi1\OneDrive - Arizona State University\Projects\Adaptive\Bayesian Optimization\Experiment_rangecheck\Data\ResultRange.mat'],'Weight','maxmin');




