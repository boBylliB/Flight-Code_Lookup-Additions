clear
c.phi = 0; % the angle counterclockwise from the y axis to the closest structural mount point, in degrees
c.count = 3; % the number of linear actuators
c.w = 3; % the planar distance from the central axis to each structural mount point
c.h = 3; % the vertical distance from the structural mount points to the motor mount points
c.d = 0.5; % the vertical distance (positive being downwards) from the structural mount points to the central motor pivot
c.r = 0.5; % the radius from the central axis to the motor mount points
P = linspace(-45,45,11); % A vector of pitch angles, in degrees
Y = P; % A vector of yaw angles, in degrees
% Pitch vector, Yaw vector, Constants, Pitch index, Yaw index, Resolution
%displayPosition(P, Y, c, 1, 1, 11)
%displayPosition(P, Y, c, 6, 6, 11)
%displayPosition(P, Y, c, 11, 6, 11)
%displayPosition(P, Y, c, 3, 7, 101)
theta = linspace(0,360,201);
R = 45*sind(theta/2);
P = R.*sind(theta*2);
Y = R.*cosd(theta*2);
%animatePosition(P, Y, c, 11, 1, 5)
P = linspace(-45,45,101);
Y = P;
tic
[L1, ~, ~] = generateTable(P, Y, c);
toc
fprintf("Pitch Diff: %f\n", P(2)-P(1));
L1 = 50 .* L1 ./ max(max(max(L1)));
writeTableToFile("TestLengthTable.testing", "Created by Sam Sandelin", P, Y, c, L1);

function displayPosition(P, Y, c, pitchIdx, yawIdx, resolution)
    f = figure('Visible', 'on');
    hold on
    theta = linspace(0,360,resolution);
    % Calculate x, y, and z points for a motor centerline
    center = [0 0 -c.d;0 0 -c.d-c.h/8;0 0 -c.h];
    rotatedCenter = [center(1,:);
                     rotatePoint(center(1,:), center(2,:), P(pitchIdx), Y(yawIdx));
                     rotatePoint(center(1,:), center(3,:), P(pitchIdx), Y(yawIdx))];
    for idx = 1:resolution
        % Draw structural ring
        if idx < resolution
            plot3([c.w * sind(theta(idx)) c.w * sind(theta(idx+1))], [c.w * cosd(theta(idx)) c.w * cosd(theta(idx+1))], [0 0])
        end
        % Calculate x, y, and z positions for points offset at r from the
        % motor centerline
        offsets = center + [c.r*sind(theta(idx)) c.r*cosd(theta(idx)) 0];
        % Rotate those points
        rotatedOffsets = [rotatePoint(center(1,:), offsets(1,:), P(pitchIdx), Y(yawIdx));
                          rotatePoint(center(1,:), offsets(2,:), P(pitchIdx), Y(yawIdx));
                          rotatePoint(center(1,:), offsets(3,:), P(pitchIdx), Y(yawIdx))];
        % Draw motor mount cone
        plot3([center(1,1) rotatedOffsets(2,1)], [center(1,2) rotatedOffsets(2,2)], [center(1,3) rotatedOffsets(2,3)])
        % Draw motor cylinder
        plot3([rotatedCenter(2,1) rotatedOffsets(2,1)], [rotatedCenter(2,2) rotatedOffsets(2,2)], [rotatedCenter(2,3) rotatedOffsets(2,3)])
        plot3([rotatedOffsets(2,1) rotatedOffsets(3,1)], [rotatedOffsets(2,2) rotatedOffsets(3,2)], [rotatedOffsets(2,3) rotatedOffsets(3,3)])
        plot3([rotatedCenter(3,1) rotatedOffsets(3,1)], [rotatedCenter(3,2) rotatedOffsets(3,2)], [rotatedCenter(3,3) rotatedOffsets(3,3)])
    end
    % Draw linear actuator lines
    [L, motorMounts, structuralMounts] = generateTable(P, Y, c);
    for idx = 1:c.count
        plot3([structuralMounts(idx,1) motorMounts(idx,pitchIdx,yawIdx,1)],[structuralMounts(idx,2) motorMounts(idx,pitchIdx,yawIdx,2)],[structuralMounts(idx,3) motorMounts(idx,pitchIdx,yawIdx,3)])
    end
    % Label axes & formatting
    axis equal
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    % Print linear actuator lengths
    for idx = 1:c.count
        fprintf("Actuator %d has a length of %f\n", idx, L(idx, pitchIdx, yawIdx));
    end
end
function animatePosition(P, Y, c, resolution, loops, loopDuration)
    f = figure('Visible', 'on');
    hold on
    theta = linspace(0,360,resolution);
    % Calculate x, y, and z points for a motor centerline
    center = [0 0 -c.d;0 0 -c.d-c.h/8;0 0 -c.h];
    rotatedCenter = [center(1,:);
                     rotatePoint(center(1,:), center(2,:), P(1), Y(1));
                     rotatePoint(center(1,:), center(3,:), P(1), Y(1))];
    for idx = 1:resolution
        % Draw structural ring
        if idx < resolution
            plot3([c.w * sind(theta(idx)) c.w * sind(theta(idx+1))], [c.w * cosd(theta(idx)) c.w * cosd(theta(idx+1))], [0 0])
        end
        % Calculate x, y, and z positions for points offset at r from the
        % motor centerline
        offsets = center + [c.r*sind(theta(idx)) c.r*cosd(theta(idx)) 0];
        % Rotate those points
        rotatedOffsets = [rotatePoint(center(1,:), offsets(1,:), P(1), Y(1));
                          rotatePoint(center(1,:), offsets(2,:), P(1), Y(1));
                          rotatePoint(center(1,:), offsets(3,:), P(1), Y(1))];
        % Draw motor mount cone
        conePlots(idx) = plot3([center(1,1) rotatedOffsets(2,1)], [center(1,2) rotatedOffsets(2,2)], [center(1,3) rotatedOffsets(2,3)]);
        % Draw motor cylinder
        cylTopPlots(idx) = plot3([rotatedCenter(2,1) rotatedOffsets(2,1)], [rotatedCenter(2,2) rotatedOffsets(2,2)], [rotatedCenter(2,3) rotatedOffsets(2,3)]);
        cylWallPlots(idx) = plot3([rotatedOffsets(2,1) rotatedOffsets(3,1)], [rotatedOffsets(2,2) rotatedOffsets(3,2)], [rotatedOffsets(2,3) rotatedOffsets(3,3)]);
        cylBottomPlots(idx) = plot3([rotatedCenter(3,1) rotatedOffsets(3,1)], [rotatedCenter(3,2) rotatedOffsets(3,2)], [rotatedCenter(3,3) rotatedOffsets(3,3)]);
    end
    % Draw linear actuator lines
    [~, motorMounts, structuralMounts] = generateTable(P, Y, c);
    for idx = 1:c.count
        actuatorPlots(idx) = plot3([structuralMounts(idx,1) motorMounts(idx,1,1,1)],[structuralMounts(idx,2) motorMounts(idx,1,1,2)],[structuralMounts(idx,3) motorMounts(idx,1,1,3)]);
    end
    % Label axes & formatting
    axis equal
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    % Animate motion
    stepDelay = loopDuration / length(P);
    for loopIdx = 1:loops
        for angleIdx = 1:length(P)
            rotatedCenter = [center(1,:);
                             rotatePoint(center(1,:), center(2,:), P(angleIdx), Y(angleIdx));
                             rotatePoint(center(1,:), center(3,:), P(angleIdx), Y(angleIdx))];
            for plotIdx = 1:resolution
                % Calculate x, y, and z positions for points offset at r from the
                % motor centerline
                offsets = center + [c.r*sind(theta(plotIdx)) c.r*cosd(theta(plotIdx)) 0];
                % Rotate those points
                rotatedOffsets = [rotatePoint(center(1,:), offsets(1,:), P(angleIdx), Y(angleIdx));
                                  rotatePoint(center(1,:), offsets(2,:), P(angleIdx), Y(angleIdx));
                                  rotatePoint(center(1,:), offsets(3,:), P(angleIdx), Y(angleIdx))];
                % Draw motor mount cone
                conePlots(plotIdx).XData = [center(1,1) rotatedOffsets(2,1)];
                conePlots(plotIdx).YData = [center(1,2) rotatedOffsets(2,2)];
                conePlots(plotIdx).ZData = [center(1,3) rotatedOffsets(2,3)];
                % Draw motor cylinder
                cylTopPlots(plotIdx).XData = [rotatedCenter(2,1) rotatedOffsets(2,1)];
                cylTopPlots(plotIdx).YData = [rotatedCenter(2,2) rotatedOffsets(2,2)];
                cylTopPlots(plotIdx).ZData = [rotatedCenter(2,3) rotatedOffsets(2,3)];
                cylWallPlots(plotIdx).XData = [rotatedOffsets(2,1) rotatedOffsets(3,1)];
                cylWallPlots(plotIdx).YData = [rotatedOffsets(2,2) rotatedOffsets(3,2)];
                cylWallPlots(plotIdx).ZData = [rotatedOffsets(2,3) rotatedOffsets(3,3)];
                cylBottomPlots(plotIdx).XData = [rotatedCenter(3,1) rotatedOffsets(3,1)];
                cylBottomPlots(plotIdx).YData = [rotatedCenter(3,2) rotatedOffsets(3,2)];
                cylBottomPlots(plotIdx).ZData = [rotatedCenter(3,3) rotatedOffsets(3,3)];
            end
            for idx = 1:c.count
                actuatorPlots(idx).XData = [structuralMounts(idx,1) motorMounts(idx,angleIdx,angleIdx,1)];
                actuatorPlots(idx).YData = [structuralMounts(idx,2) motorMounts(idx,angleIdx,angleIdx,2)];
                actuatorPlots(idx).ZData = [structuralMounts(idx,3) motorMounts(idx,angleIdx,angleIdx,3)];
            end
            pause(stepDelay);
        end
    end
end
function newPoint = rotatePoint(origin, point, pitch, yaw)
    offset = point - origin;
    pitchInitial = atan2d(offset(2),offset(3));
    yawInitial = atan2d(offset(1),offset(3));
    rXZ = sqrt(offset(1)^2 + offset(3)^2);
    rYZ = sqrt(offset(2)^2 + offset(3)^2);
    pitchFinal = pitchInitial + pitch;
    yawFinal = yawInitial + yaw;
    newPoint(1) = rXZ * sind(yawFinal) + origin(1);
    newPoint(2) = rYZ * sind(pitchFinal) + origin(2);
    newPoint(3) = rYZ * cosd(pitchFinal) + rXZ * cosd(yawFinal) + origin(3);
end
function [L, motorMounts, structuralMounts] = generateTable(P, Y, c)
    % We define the y axis as pointing in the same direction as the Pitch
    % direction, and the x axis as pointing in the same direction as the
    % Yaw direction
    % Linear actuator 1 is the closest structural mount point, moving
    % counterclockwise, from a desired positive pitch direction
    % Linear actuators are then numbered in clockwise order, from the
    % perspective of looking down on the structure (or counterclockwise
    % from the perspective of someone looking at the bottom of the rocket,
    % with linear actuator 1 then being the closest linear actuator, moving
    % clockwise, from the positive pitch direction)
    % c, our constants struct, contains the following:
    %   phi =   the angle counterclockwise from the y axis to the closest
    %           structural mount point, in degrees
    %   count = the number of linear actuators
    %   w =     the planar distance from the central axis to each
    %           structural mount point
    %   h =     the vertical distance from the structural mount points to
    %           the motor mount points
    %   d =     the vertical distance from the structural mount points to
    %           the motor pivot point, positive being downwards
    %   r =     the radius from the central axis to the motor mount points
    % We then need to calculate the angle from the y axis, clockwise, to
    % each linear actuator's structural mount point
    theta = zeros([c.count 1]);
    for idx = 1:c.count
        theta(idx) = (360 / c.count)*(idx-1) - c.phi;
    end
    % Next, we can use that and the mounting width to calculate the x and y
    % coordinates of each structural mount point, with z being 0
    structuralMounts = zeros([c.count 3]);
    for idx = 1:c.count
        structuralMounts(idx, 1) = c.w * sind(theta(idx)); % x
        structuralMounts(idx, 2) = c.w * cosd(theta(idx)); % y
    end
    % We then use the mounting height, angle, and motor radius to calculate
    % the x, y, and z coordinates of each motor mount point
    centralMotorMounts = zeros([c.count 3]);
    for idx = 1:c.count
        centralMotorMounts(idx, 1) = c.r * sind(theta(idx)); % x
        centralMotorMounts(idx, 2) = c.r * cosd(theta(idx)); % y
        centralMotorMounts(idx, 3) = -c.h; % z
    end
    % Once we have those "central" coordinates, we can iterate over every
    % possible pitch and yaw to rotate those points
    motorMounts = zeros([c.count length(P) length(Y) 3]);
    for idx = 1:c.count
        for pitchIdx = 1:length(P)
            for yawIdx = 1:length(Y)
                motorMounts(idx,pitchIdx,yawIdx,:) = rotatePoint([0 0 -c.d], centralMotorMounts(idx,:), P(pitchIdx), Y(yawIdx));
            end
        end
    end
    % Finally, all of those motor mount coordinates can be used with the
    % structural mount coordinates to get length targets for every linear
    % actuator
    L = zeros([c.count length(P) length(Y)]);
    difference = zeros([length(P) length(Y) 3]);
    for idx = 1:c.count
        for pitchIdx = 1:length(P)
            for yawIdx = 1:length(Y)
                difference(pitchIdx,yawIdx,1) = abs(motorMounts(idx,pitchIdx,yawIdx,1) - structuralMounts(idx,1));
                difference(pitchIdx,yawIdx,2) = abs(motorMounts(idx,pitchIdx,yawIdx,2) - structuralMounts(idx,2));
                difference(pitchIdx,yawIdx,3) = abs(motorMounts(idx,pitchIdx,yawIdx,3) - structuralMounts(idx,3));
                L(idx,pitchIdx,yawIdx) = sqrt(difference(pitchIdx,yawIdx,1)^2 + difference(pitchIdx,yawIdx,2)^2 + difference(pitchIdx,yawIdx,3)^2);
            end
        end
    end
end
function writeTableToFile(filename, header, P, Y, c, L)
    % NOTE: Any changes to this function should also be accompanied by
    % changes or a review of the corresponding arduino file logic to make
    % sure everything still lines up
    % The output file will have a header, in which the final line will
    % be a line of equal signs, promptly followed by fixed-length file
    % information for the arduino code
    % The header consists of the given descriptive header, combined with an
    % auto-generated header listing the constants that the table was
    % generated for, as a double-checking step
    % This line of equals sign should be avoided in writing the header, and
    % therefore simply checking for an equals character in a line can be
    % used to detect the end of the header
    % WARNING: This function WILL overwrite the filename given
    % The output format of the main data section of the file is as follows:
    % <sign><pitchAngle %2.3f> <sign><yawAngle %2.3f> <length1 %2d> <length2 %2d> ...
    packedLineSize = 4 + c.count;

    if ~contains(filename, ".txt")
        filename = strcat(filename, ".txt");
    end
    fileID = fopen(filename, 'w');
    
    % Header
    fprintf(fileID, header);
    if ~isempty(header) && ~endsWith(header, '\n')
        fprintf(fileID,'\n');
    end
    fprintf(fileID, '%s\n', repelem('-',1,50));
    fprintf(fileID, "Length Table generated on %s\n", string(datetime));
    fprintf(fileID, "The constants struct utilized the following:\n- phi\t-> %f\n- count\t-> %d\n- w\t-> %f\n- h\t-> %f\n- d\t-> %f\n- r\t-> %f\n", ...
                                                                   c.phi,     c.count,         c.w,       c.h,       c.d,       c.r);
    fprintf(fileID, "Pitch ranges from %f to %f\nYaw ranges from   %f to %f",min(P), max(P), min(Y), max(Y));
    fprintf(fileID, '\n%s\n', repelem('=',1,50));

    % File Information
    % <pitch length> <pitch min> <pitch max>
    % <yaw length> <yaw min> <yaw max>
    fprintf(fileID, "%d %06.3f %06.3f\n%d %06.3f %06.3f\n", length(P), min(P), max(P), length(Y), min(Y), max(Y));

    % Data
    for pitchIdx = 1:length(P)
        for yawIdx = 1:length(Y)
            if P(pitchIdx) < 0
                fprintf(fileID, "-");
            else
                fprintf(fileID, " ");
            end
            fprintf(fileID, "%06.3f ", abs(P(pitchIdx)));
            if Y(yawIdx) < 0
                fprintf(fileID, "-");
            else
                fprintf(fileID, " ");
            end
            fprintf(fileID, "%06.3f", abs(Y(yawIdx)));
            for actuatorIdx = 1:c.count
                fprintf(fileID, " %02d", round(L(actuatorIdx,pitchIdx,yawIdx)));
            end
            fprintf(fileID, "\n");
        end
    end

    fclose(fileID);

    % Calculate estimated data size
    fprintf("Estimated table size once loaded: %d kB\n", ceil(packedLineSize*length(P)*length(Y)/1024));
    fprintf("Estimated compressed table size:  %d kB\n", ceil(c.count*length(P)*length(Y)/1024));
end