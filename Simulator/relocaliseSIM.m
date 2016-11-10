function [botSim, botEst, particles] = relocaliseSIM(botSim, map, target, particles, particle_data, drawing, debug)
% Relocalisation
% By Asher Winterson & Aidan Scannell
%
%	===== Inputs =====
%	botSim 	- BotSim class
%	map		- list of vertices of map polygon
%	target 	- [x, y] coordinates of the target
%	particles	- Array of all the particles generated
%	Turn		- List of bot turns
%	Forward		- List of bot forward movements
%	drawing - outputs a graphical result
%	debug 	- outputs a detailed graphical output for debugging
%
%	===== Outputs ====
%	botSim 		- BotSim class
%	botEst 		- Estimated bot position and heading
%	particles	- Array of all the particles generated

mapArea = polyarea(map(:,1),map(:,2)); %Find area of map
num = round((-3e-7*mapArea^2)+(0.02*mapArea)+230);
if mapArea <= 1600
    tol = 1;
else
    tol = round(0.0001*mapArea+0.87);
end

wallClearance = 7; % wall clearance of bot
numberScans = 20; % number of ultrasound scans
botSim.setScanConfig(botSim.generateScanConfig(numberScans));

%% Movement
% botSim.turn(Turn); %Move Bot
% botSim.move(Forward); %Move Bot
% particle_data = zeros(num, 3);
% for ii =1:num
%     particles(ii).turn(Turn); %move particles
%     particles(ii).move(Forward); %Move particles
%     if particles(ii).insideMap == 0 % if inside map resample
%         particles(ii).randomPose(wallClearance);
%         %weights(ii) = 0; % if fail set to 0
%         %         try
%         %             y = randsample(1:num,num,true,weights); %resample
%         %         catch
%         %             weights(ii) = 0; % if fail set to 0
%         %         end
%         %         particles(ii).setBotPos(particles(y(ii)).getBotPos())
%         %         particles(ii).setBotAng(particles(y(ii)).getBotAng())
%     end
%     particle_data(ii,1:2) = particles(ii).getBotPos();
%     particle_data(ii,3) = mod(particles(ii).getBotAng(),2*pi);
% end

%% Update and Score Particles

botScan = botSim.ultraScan(); %botSim Scan
weights = zeros(num, 1);
for ii = 1 : num
    particle_scan = particles(ii).ultraScan; %particle scan
    d = zeros(numberScans,1);
    for jj = 1 : numberScans
        d(jj) = sqrt(sum((particle_scan-botScan).^2)); %euclidean distance
        particle_scan = circshift(particle_scan,-1); %repeat at every orientation
    end
    [min_d, min_d_ind] = min(d); %find minimum euclidean distance (ED) to select correct orientation
    turn = (min_d_ind-1)*(2*pi()/numberScans); %set particle turning distance
    weights(ii) = 1/min_d; %use min ED of selected orientation to obtain weightings
    particles(ii).turn(turn) %Move particles to correct orientation
end
weights = weights/sum(weights); %normalize

%% Resampling - Resampling Wheel

%Initialize variables
index = randi([1, num-1]);  %random number for initial starting point on wheel
beta = 0;
max_weight = max(weights);
for ii = 1 : num
    beta = beta + rand(1)*2*max_weight; %aidan input description here!
    while beta > weights(index) %aidan input description here!
        beta = beta - weights(index); %aidan input description here!
        index = rem((index+1),num)+1; %aidan input description here!
        weights(ii) = weights(index);%aidan input description here!
        particle_to_copy = index; %aidan input description here!
        particles(ii).setBotPos(particles(particle_to_copy).getBotPos()); %aidan input description here!
        particles(ii).setBotAng(particles(particle_to_copy).getBotAng());%aidan input description here!
    end
end




%% Convergence
%By using 'uniquetol' clusters of particles can be found by setting...
%'tol' to an appropriate value to increase required accuracy before...
%convergence.
% 'DataScale' is required to scale the angle data to work with the
% single tolerance level

[C,iA] = uniquetol(particle_data(:,1:3),tol,'ByRows',true,'OutputAllIndices',true,'DataScale',[1,1,(180/pi)]);

[~,weight_max_ind] = max(cellfun('size', iA, 1)); %find the max cell size
botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
botEst(3) = mod(botEst(3), 2*pi); %retrieve estimate for bot position


%% Drawing
if drawing == 1
    hold off; %the drawMap() function will clear the drawing when hold is off
    axis equal
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    axis manual
    botSim.drawScanConfig();
    plot(target(1),target(2),'*b', 'MarkerSize', 5);
    
    if debug == 1
        plot(particle_data(:,1), particle_data(:,2), '.k', 'MarkerSize', 7);
        %for ii =1:num
        %    particles(ii).drawBot(weights(ii)*1000);
        %end
    end
    
    if isnan(botEst(1))
    else
        if botEst(3) <= pi
            heading = [botEst(1), botEst(2);
                botEst(1) + 10*cos(botEst(3)), botEst(2)+10*sin(botEst(3))];
        elseif botEst(3) > pi && botEst(3) <= 2*pi
            botEst(3) = botEst(3) - 2*pi;
            heading = [botEst(1), botEst(2);
                botEst(1) + 10*cos(botEst(3)), botEst(2)+10*sin(botEst(3))];
        end
        plot(botEst(1), botEst(2), '.r', 'MarkerSize', 20);
        plot(heading(:,1), heading(:,2), 'lineWidth',1.5,'Color','r');
    end

    botSim.drawBot(10,'b');
    hold on
    drawnow;
end


end
