function [ botSim, particles, particle_data ] = movement( botSim, particles, wallClearance, Turn, Forward )
% Moves all the particles and the bot a single turn and forward movement
%
%	===== Inputs =====
%	botSim          - BotSim class
%	particles       - Array of all the particles generated
%   wallClearance   - minimum distance from wall
%   Turn            - a turn command in rads
%   Forward         - a move command in cm
%
%	===== Outputs ====
%	botSim 	- BotSim class
%	particles       - Array of all the particles generated
%	particles_data  - Array of particles position and heading
%
% by David Mathias

num = size(particles,1);

botSim.turn(Turn); %Move Bot
botSim.move(Forward); %Move Bot
particle_data = zeros(num, 3);
for ii =1:num
    particles(ii).turn(Turn); %move particles
    particles(ii).move(Forward); %Move particles
    if particles(ii).insideMap == 0 % if inside map resample
        particles(ii).randomPose(wallClearance);
    end
    particle_data(ii,1:2) = particles(ii).getBotPos();
    particle_data(ii,3) = mod(particles(ii).getBotAng(),2*pi);
end

end

