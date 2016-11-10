function [ botSim ] = localise( botSim, map, target )
%	Localistation function for simulated robot. Group MMSW-1.
%   
%   by David Mathias, Max Martin, Aidan Scannell & Asher Winterson
%
%	===== Inputs =====
%	botSim 	- BotSim class
%	map		- list of vertices of map polygon
%	target 	- [x, y] coordinates of the target

%	===== Outputs ====
%	botSim 	- BotSim class

drawing = true; % Outputs graphical displays for results
debug = true; 	% Outputs graphical displays for debugging

if drawing == true
    axis equal
    axis manual
    botSim.drawMap(); 				% Draw map
    botSim.drawBot(3); 				% Draw initial bot position
    plot(target(1),target(2),'*');	% Draw target
end

%% --- Initial localisation ---
lost = 1;
while lost == 1 % Loops through localisation until the bot has an estimated position
    % First localisation - estimates bots position and heading
    [botSim, botEst, particles, lost] = sim_localiseSIM(botSim, map, target, drawing, debug);
end

start = [botEst(1), botEst(2)]; % Start position for route planner
start_angle = botEst(3);		% Start heading

%% --- Route planning ---
wallClearance = 7;
resolution = 1; 				% Resolution of grid used for route planning
mink = minkowski(5, map, 0);	% Inflates map to avoid wall collisions

% Generates a discretised map for route planning
[mapGrid, limsMin, limsMax] = inMap(map, mink, resolution, 0);

% Plans a route from the current bot estimation to the target
[botSim, moves] = routePlanSIM(botSim, mapGrid, limsMin, start, target, resolution, drawing);

% Converts the route into bot moves
[botSim, Turn, Forward] = move_commandSIM( botSim, moves, start_angle, resolution);

%% --- Relocalisation ---
distToTarget = sqrt((botEst(1)-target(1))^2 + (botEst(2)-target(2))^2); % Pythagorean distance to target
maxDistInMap = sqrt((limsMin(1)-limsMax(1))^2 + (limsMin(2)-limsMax(2))^2);

while distToTarget > 2.5 && distToTarget < maxDistInMap % Loops until the estimate bot is close enough to the target
    
    if isempty(Turn) % Checks for move commands
        % if no commands, generates arbitrary commands until a path is found
        Turn(1) = mod(start_angle - pi, 2*pi);
        Forward(1) = 5;
        [botSim, particles, particle_data] = movement(botSim, particles, wallClearance, Turn(1), Forward(1));
    
    elseif distToTarget > 10 && ~isempty(Turn) % checks distance to target and for move commands
        distMoved = 0;
        while distMoved < 10 % makes consecutive moves for 10cm
            [botSim, particles, particle_data] = movement(botSim, particles, wallClearance, Turn(1), Forward(1));
            distMoved = distMoved + Forward(1); % increments distance moved
            Turn(1) = [];       % removes movement used
            Forward(1) = [];    % removes movement used
            if isempty(Turn)    
                break           % breaks if no moves left
            end
        end
        
    elseif ~isempty(Turn) % excutes smaller moves when close to target
        [botSim, particles, particle_data] = movement(botSim, particles, wallClearance, Turn(1), Forward(1));
    end
    
    % Scans and generates an updated bot estimate after moving
    [botSim, botEst, particles] = relocaliseSIM(botSim, map, target, particles, particle_data, drawing, debug);
    
    start = [botEst(1), botEst(2)]; % Current bot estimate for route planner
    start_angle = botEst(3);		% Current heading estimate
    
    % Plans a new route from the current bot estimation to the target
    [botSim, moves] = routePlanSIM(botSim, mapGrid, limsMin, start, target, resolution, drawing);
    
    n = 30;
    if length(moves) > n 	% Curtails moves if too long to speed up code
        moves = moves(1:n);
        % Converts the route into bot moves
        [botSim, Turn, Forward] = move_commandSIM( botSim, moves, start_angle, resolution);
    elseif ~isempty(moves) && length(moves) <= n
        % Converts the route into bot moves
        [botSim, Turn, Forward] = move_commandSIM( botSim, moves, start_angle, resolution);
    elseif moves == 0
        Turn(1) = mod(start_angle - pi, 2*pi);
        Forward(1) = 5;
    end
    
    
    % Calculates a new distance to the target
    distToTarget = sqrt((botEst(1)-target(1))^2 + (botEst(2)-target(2))^2);
    
end

end