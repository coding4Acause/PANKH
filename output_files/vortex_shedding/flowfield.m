clear
close all 
Files = dir('motion*.dat');
Wake_files = dir('wake*.dat');

% Find the total number of JPEG files in the Current Folder
NumFiles = size(Files,1);

% Create a VideoWriter object to write video frames
v = VideoWriter("vortex_Shedding","MPEG-4");
open(v)
figure1 = figure('units','pixels','position',[0 0 2160 360]); % Frame size 960x720
axes('Parent',figure1, FontSize=30);
% Assuming your data has two columns (X and Y)

    for i = 1 : NumFiles-1
        pitch_datas = sprintf('motion_%d.dat', i);
        wake_datas  =  sprintf('wake_%d.dat', i);
        data1 = load(pitch_datas);
        data2 = load(wake_datas);
    
        % Load data
        X = data1(:, 1);
        Y = data1(:, 2);
        X1 = data2(:, 1);
        Y1 = data2(:, 2);
        
        % Plot data
        plot(X, Y, '-',X1, Y1, '.',MarkerSize=10); % Use 'o-' for a line with markers
        
        % % Fixed minimum limits
        % min_x = 0;
        % 
        % % Dynamic maximum limits
        % max_x = max(max(X), max(X1));
        % min_y = min(min(Y), min(Y1));
        % max_y = max(max(Y), max(Y1));
        % 
        fixed_min_x = 0;
        fixed_max_x = 3;  % Adjust according to your data range
        fixed_min_y = -0.35; % Adjust according to your data range
        fixed_max_y = 0.35;  % Adjust according to your data range
            % Set limits with fixed minimums and dynamic maximums
        xlim([fixed_min_x, fixed_max_x])
        ylim([fixed_min_y, fixed_max_y])
        
        title('pitching and plunging naca 0012');
        xlabel('x',FontSize=30);
        ylabel('y',FontSize=30);
        % axis equal; % Ensure equal scaling on both axes
        grid on;
        drawnow;
        
        % Capture the plot as a frame and write it to the video
        frame = getframe(gcf);
        writeVideo(v, frame);
    end


% Close the video file
close(v);
