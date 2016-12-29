function [ botSim, Turn, Forward] = move_commandSIM( botSim, moves, start_angle, resolution)
% Converts the discrete route into bot moves
%
%	===== Inputs =====
%	botSim 		- BotSim Class
%	moves 		- List of unit moves from start to target
%	start_angle - start heading of bot
%	resolution 	- size of route steps
%	===== Outputs ====
%	botSim 		- BotSim Class
%	Turn		- List of bot turns
%	Forward		- List of bot forward movements
%	

% set initial values
ii = 1;
sum_moves = 0;
count1 = 1;
count2 = 1;
Forward = zeros(size(moves,1)); % preallocation for speed

% change direction to face direction 1
if start_angle ~= pi/2;
    Turn(1) = -start_angle+(pi/2);
end

% start loop
finish = length(moves);
for i = 1:finish;

    % break on last iteration
    if finish == ii + sum_moves - 1;

        break
    end

    %%% Rotation for first iteration %%%
    if i == 1;
        Turn(count1) = Turn(count1)+(pi/4)*moves(i)-(pi/4);
        count1 = count1+1;

    %%% Rotation after first iteration %%%
    else
        % adjust iterations
        ii = ii + sum_moves;
        i = ii;
        % rotate
        if moves(i-1) < 5;
            if moves(i) > moves(i-1) && moves(i) < (moves(i-1)+5);
                % Turn Left
                diff = abs(moves(i)-moves(i-1));
                Turn(count1) = (pi/4)*diff;
                count1 = count1 +1;
            else
                % turn right
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    Turn(count1) = (-pi/4)*diff;
                    count1 = count1 + 1;
                else
                    Turn(count1) = (-pi/4)*diff;
                    count1 = count1 + 1;
                end
            end
        else
            if moves(i) > (moves(i-1)-4) && moves(i) < moves(i-1);
                % turn right
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    Turn(count1) = (-pi/4)*diff;
                    count1 = count1 + 1;
                else
                    Turn(count1) = (-pi/4)*diff;
                    count1 = count1 + 1;
                end
            else
                % turn left
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;

                    Turn(count1) = (pi/4)*diff;
                    count1 = count1 + 1;
                else

                    Turn(count1) = (pi/4)*diff;
                    count1 = count1 + 1;
                end
            end
        end
    end

    %%% movement %%%
    % sum moves
    ii = i;
    sum_moves = 0;
    n = moves(i);
    while moves(i)== n;
        sum_moves = sum_moves+1;
        if i == finish
            break
        end
        i=i+1;
    end
    i = ii;

    %diagonal add distance
    if n == 2 || n == 4 || n == 6 || n == 8;
        Forward(count2) = sum_moves*resolution*1.4141;
        count2 = count2 +1;
    else

        Forward(count2) = sum_moves*resolution;
        count2 = count2 +1;
    end
end

Forward = Forward(Forward > 0);


end
