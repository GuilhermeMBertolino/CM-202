function plotOrientations(x, y, psi, step)
% plotOrientations(x, y, psi, step) plots the robot's orientations in a
% trajectory. The vectors x, y, and psi encode the robot's pose along the
% trajectory. Moreover, step is used to provide a sparser representation of
% the robot's orientation along the trajectory.

ARROW_SIZE = 0.07;

for i=1:step:length(x)
    drawArrow([x(i), y(i)], [x(i), y(i)] + ARROW_SIZE * [cos(psi(i)), sin(psi(i))], 'g');
end

end