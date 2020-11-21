clc
clear
%close all

subnum = 6;



errorbound = 0.005;
for direction = 1 : 2
    if direction == 1
        block = [5 9 10 12 13];
    else
        block = [6 7 8 11 14];
    end
    for ind_block = 1: length(block)
        clear measures
        for trial = 1 : 10
            clear data DATA move hit ind_stability
            str = ['C:\Users\fzahedi1\OneDrive - Arizona State University\Projects\Adaptive\Bayesian Optimization\Experiment_rangecheck\Data\Subject',num2str(subnum),'\Direction',num2str(direction),'\Block',num2str(block(ind_block)),'\Trial',num2str(trial),'\KukaData.txt'];
            data = load(str);

            DATA.x = data(:,27);
            DATA.y = data(:,28);
            DATA.xinit = data(:,25);
            DATA.yinit = data(:,26);
            DATA.xtarget = data(:,23);
            DATA.ytarget = data(:,24);
            DATA.Vx = data(:,20);
            DATA.Vy = -data(:,19);
            DATA.ax = data(:,22);
            DATA.ay = -data(:,21);
            DATA.uintentx = DATA.Vx.*DATA.ax;
            DATA.uintenty = DATA.Vy.*DATA.ay;
            DATA.Bx = data(:,18);
            DATA.By = data(:,17);
            DATA.Forcex = data(:,10);
            DATA.Forcey = -data(:,9);
            DATA.bLB = data(:,37);
            if direction == 1
                DATA.bLB = data(:,37);
            else
                DATA.bLB = data(:,38);
            end

            move = find(((DATA.x-DATA.xinit(10)).^2+(DATA.y-DATA.yinit(10)).^2)>= (errorbound)^2);
            flag = 0;
            q = 1;
            for f=1:length(move)
                    if f < length(move) && flag == 0

                        if move(f+1)-move(f) > 1
                            e_length = f - q;
                            if e_length >= 500
                                firstmove = move(q);
                                flag = 1;
                            else
                                q = f+1;
                            end
                        end
                    end
                    if f == length(move) && flag == 0
                        firstmove = move(q);
                    end
            end

            hit = find(((DATA.x-DATA.xtarget(10)).^2+(DATA.y-DATA.ytarget(10)).^2)<= (errorbound)^2);
            firsthit = hit(1);
            if firstmove == 1
                testtrial = trial;
            end
            ind_stability = find(((DATA.x-DATA.xtarget(10)).^2+(DATA.y-DATA.ytarget(10)).^2)<= (errorbound)^2);
            flag = 0;
            q = 1;
            for f=1:length(ind_stability)
                    if f < length(ind_stability) && flag == 0

                        if ind_stability(f+1)-ind_stability(f) > 1
                            e_length = f - q;
                            if e_length >= 500
                                tstability = ind_stability(q);
                                flag = 1;
                            else
                                q = f+1;
                            end
                        end
                    end
                    if f == length(ind_stability) && flag == 0
                        e_length = f - q;
                        if e_length >= 500
                            tstability = ind_stability(q);
                        else
                            tstability = 2000 + firsthit;
                        end
                    end
            end

            % finding overshoot

            clear xrotated yrotated

            angleofrotation = atan2(DATA.ytarget(10)-DATA.yinit(10),DATA.xtarget(10)-DATA.xinit(10));
            angle = -angleofrotation;
            xrotated = (DATA.x-DATA.xinit(10))*cos(angle)-(DATA.y-DATA.yinit(10))*sin(angle);
            yrotated = (DATA.y-DATA.yinit(10))*cos(angle)+(DATA.x-DATA.xinit(10))*sin(angle);
            xtargetrotated = (DATA.xtarget(10)-DATA.xinit(10))*cos(angle)-(DATA.ytarget(10)-DATA.yinit(10))*sin(angle);
            ytargetrotated = (DATA.ytarget(10)-DATA.yinit(10))*cos(angle)+(DATA.xtarget(10)-DATA.xinit(10))*sin(angle);

            tangentialovershootpercentage = abs((max(abs(xrotated))-xtargetrotated)/xtargetrotated)*100;
            tangentialovershoot_0 = abs((max(abs(xrotated))-abs(xtargetrotated)));% - errorbound;
%             if tangentialovershoot_0 <= 0
%                 tangentialovershoot = 0;
%             else
%                 tangentialovershoot = tangentialovershoot_0;
%             end
            normalovershoot_0 = abs((max(abs(yrotated))-abs(ytargetrotated)));% - errorbound;
%             if normalovershoot_0 <= 0
%                 normalovershoot = 0;
%             else
%                 normalovershoot = normalovershoot_0;
%             end
            
            tangentialovershoot = tangentialovershoot_0;
            normalovershoot = normalovershoot_0;


            measures(trial,1) = tangentialovershoot;
            measures(trial,2) = normalovershoot; % not useful at this moment


            %mean speed
            if direction == 1
                velocity = abs(DATA.Vx);
            else
                velocity = abs(DATA.Vy);
            end
            meanspeed = mean(velocity(firstmove:firsthit));

            measures(trial,3) = meanspeed;

            % maximum speed

            maxspeed = max(velocity);
            measures(trial,5) = maxspeed;

            % User Effort

            ForceRMS = sqrt(0.5*(DATA.Forcex.^2 + DATA.Forcey.^2));

            meanRMS = mean(ForceRMS);
            maxRMS = max(ForceRMS);

            measures(trial,4) = meanRMS;
            measures(trial,6) = maxRMS;

        end
        b_LB = DATA.bLB;
        block_measures(ind_block,1:6,direction) = mean(measures);
        block_measures(ind_block,7,direction) = b_LB(10);

    end
end

[~,order] = sortrows(block_measures(:,:,1),7);
ML_measures = block_measures(order,:,1);
clear order
[~,order] = sortrows(block_measures(:,:,2),7);
AP_measures = block_measures(order,:,2);


%%

figure
subplot(3,2,1)
plot(ML_measures(:,end),ML_measures(:,1),'LineWidth',1.5)
title('ML');
ylabel('Overshoot');
xticks([]);
subplot(3,2,2)
plot(AP_measures(:,end),AP_measures(:,1),'LineWidth',1.5)
xticks([]);
title('AP');
subplot(3,2,3)
plot(ML_measures(:,end),ML_measures(:,3),'LineWidth',1.5)
xticks([]);
ylabel('MeanSpeed');
subplot(3,2,4)
plot(AP_measures(:,end),AP_measures(:,3),'LineWidth',1.5)
xticks([]);
subplot(3,2,5)
plot(ML_measures(:,end),ML_measures(:,4),'LineWidth',1.5)
ylabel('MeanRMS_{force}');
xlabel('b_{LB}');
subplot(3,2,6)
plot(AP_measures(:,end),AP_measures(:,4),'LineWidth',1.5)
xlabel('b_{LB}');

%%

ave_ML = mean(ML_measures(:,1:6));
st_ML = std(ML_measures(:,1:6));

ave_AP = mean(AP_measures(:,1:6));
st_AP = std(AP_measures(:,1:6));

ML_range = ave_ML([1, 3, 4])';
ML_st = st_ML([1, 3, 4])';

AP_range = ave_AP([1, 3, 4])';
AP_st = st_AP([1, 3, 4])';

for i=1:length(ML_range)
    COV(i,1) = (ML_range(i)/ML_st(i))*100;
    COV(i,2) = (AP_range(i)/AP_st(i))*100; % AP direction is in the second column
end

save(['C:\Users\fzahedi1\OneDrive - Arizona State University\Projects\Adaptive\Bayesian Optimization\Experiment_rangecheck\Data\Subject',num2str(subnum),'\Measures.mat'],'ML_measures','AP_measures','COV');

figure
labels = {'OS';'Speed';'Force'};
subplot(1,2,1)
for j=1:size(ML_range,1)
bar(j,ML_range(j))
hold on
error= ML_st(j);
errorbar(j,ML_range(j),zeros(size(error)),error , '.k');
end
title('ML')
set(gca, 'XTickLabel',labels, 'XTick',1:numel(labels))

subplot(1,2,2)
for j=1:size(AP_range,1)
bar(j,AP_range(j))
hold on
error= AP_st(j);
errorbar(j,AP_range(j),zeros(size(error)),error , '.k');
end
title('AP')
set(gca, 'XTickLabel',labels, 'XTick',1:numel(labels))









    
    
        