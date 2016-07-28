%% snake_in_pipes_main
% This code makes the SEA snake crawl inside a pipe while estimating its
% orientation in an inertial frame centered at the snake center of mass
%
% Author: Elena Morara
% Date: 07/19/2016

%%
clc
close all
clear all
addpath(genpath('dependencies'))
HebiLookup

%% OPTIONS

% Do you wanna rectify the head?
RECTIFY_HEAD = 1; %1 OR 0

% Make two vector as long as the number of modules which define whether
% they are trustables
% Which accelerometers do you trust? Set their trustability to 1
accTrustability = ones(1,16);
accTrustability([2 end-2]) = 0;
% Which gyros do you trust? Set their trustability to 1
gyrosTrustability = ones(1,16);
gyrosTrustability([6 7 6 15]) = 0;

%Name one module which is on the snake
one_module_name = 'SA002';

%Gait parameters
r = 0.03; %radius [m] 
p = 0.06; %2pi*p is pitch [m]
gaitSpeed = 1/5;
if RECTIFY_HEAD
    load('fit_p006_r003_3joints')
end

%% SNAKE INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Snake initalization   %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Inititalization..')

% Connects to the snake
HebiLookup;
g = HebiLookup.newConnectedGroupFromName('*',one_module_name);
numModules = g.getInfo.numModules;
snakeData = setupSnakeData( 'SEA Snake', numModules);
cmd = CommandStruct;

% Set gain strategy
setSEASnakeGains_Strategy4;

% Loads the  offsets of the gyros and accelerometers
accelOffsets = retrieveAccelOffsets(g.getInfo().name);
gyroOffsets = retrieveGyroOffsets(g.getInfo().name);

disp('Inititalization done!')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Initial checks       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('%%%%%%% START OF THE CHECKS %%%%%%%');

%%%%% Motors 
disp('Please check the MOTORS')

% Straight snake
cmd.position = zeros(1,numModules);
cmd.torque = zeros(1,numModules);
for i=1:5
    g.set(cmd);
    pause(.25);
end

disp('Is the snake straight?')
disp('Press any key to confirm')
disp('----');
disp('')

pause();

%%%%% Accelerometers
disp('Please check ACCELEROMETERS')

% Plot the snake with the accelerometer readings
plt = HebiPlotter('frame','gravity', 'accelOffsets', accelOffsets, ...
     'gyroOffsets', gyroOffsets, 'gyrosTrustability', gyrosTrustability, ...
     'accTrustability', accTrustability);
plt.plot(g.getNextFeedback);

% Limp the snake
cmd.position = NaN(1,numModules);
cmd.velocity = NaN(1,numModules);
cmd.torque = zeros(1,numModules);
for i=1:5
    g.set(cmd);
    pause(.25);
end

disp('Blue/red vectors belong to modules whose accelerometers are currently ignored')
disp('Please roll the snake on itself')
disp('Are gravity vectors meaningful?')
disp('Press any key to confirm (while the figure is active)')
 
% Wait for a key to be pressed
global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
gcf
set(gcf, 'KeyPressFcn', @myKeyPressFcn)
 while(~KEY_IS_PRESSED)
    plt.plot(g.getNextFeedback);
 end
 
 disp('----');
 disp('')
 

 %%%%% GoPro
disp('Please check GOPRO')
disp('Is it agreeing with the smiley face?')
disp('Press any key to confirm (while the figure is active)')

 KEY_IS_PRESSED = 0;
 set(gcf, 'KeyPressFcn', @myKeyPressFcn)
while(~KEY_IS_PRESSED)
    plt.plot(g.getNextFeedback);
end
 close all
 disp('----');
 disp('') 
 
close all
 
%%%% Gyros 
disp('Please out and leave the snake still on the floor and press a key')
pause()

% Shortly computes the gyros offsets and sees how much it is consistent
% with the previously loaded one
% Also checks the standard deviation fo the gyro readings
[mean_variation, deviations] = check_gyro_offsets(g,gyroOffsets, gyrosTrustability);
disp('Please check the GYROS')


if max(max(mean_variation)) > 10
    disp('!!!!!!!!!!! WARNING !!!!!!!!! apparently some gyro offsets have changed significantly');
    disp('You may wanna recalibrate (compute_gyro_offsets(g))');
end
if max(max(deviations)) > 0.01
    disp('!!!!!!!!!!! WARNING !!!!!!!!! apparently some gyros reading are very oscillating');
    disp('You may wanna ignore these values in the Complementary Filter rhough gyroTrustability');
end


pause()
 disp('----');
 disp('')
close all


disp('Please TURN ON GOPRO WHEN IT IS FACING UP')
disp('Is it turned on?')

pause()
disp('----');
disp('') 

disp('Please REMEMBER THE LIGHT')
disp('Is it turned on?')

pause()
disp('----');
disp('') 
 
disp('%%%%%%% END OF THE CHECKS %%%%%%%');
disp(' ')
disp('You should insert the snake inside the pipe')
pause() 
 
 %% CF INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Complementary filter initalization %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CF = ComplementaryFilter(snakeData, 'accelOffsets', accelOffsets, 'gyroOffsets', gyroOffsets, 'gyrosTrustability', gyrosTrustability);

fk_CF = CF.getInGravity ('wholeBody');
VC_CF = CF.getInGravity ('VC');
while  isempty(fk_CF)
    CF.update(g.getNextFeedback());
    fk_CF = CF.getInGravity ('wholeBody');
    VC_CF = CF.getInGravity ('VC');
end 

%% GAIT INITALIZAION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gait initalization %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gaitInfo =  gaitRolling();

params = gaitInfo.defaultParams;
m = snakeData.moduleLen; %[m] 

tau = p/(r^2 + p^2);
kappa = r/(r^2 + p^2);
A = 2*kappa/tau *sin(tau*m);
tauPerM = tau*m;

params(2) = A;
params(3) = tauPerM; 


%Intial position
cmd = CommandStruct;
xDOF = snakeData.x_ang_mask;
yDOF = snakeData.y_ang_mask;
params(1) = 0;
[xangles, yangles] = gaitInfo.gaitFunction(params, snakeData);
jointAngles = nan(1,numModules);
jointAngles(xDOF) = xangles(xDOF);
jointAngles(yDOF) = yangles(yDOF);
if RECTIFY_HEAD
    j0 = approximate( 0, fit_theta0);
    jm1 = approximate( 0, fit_thetam1);
    jm2 = approximate( 0, fit_thetam2);    
    jointAngles = jointAngles(4:end);
    jointAngles = [jm2 jm1 j0 jointAngles];
end
jointAngles = anglesUtoSEA(snakeData, jointAngles) ;
thetaRef = jointAngles;     

% Slow motion of snake to inital pose %
fbk = g.getNextFeedback();
thetaFb = fbk.position; %rad  
dTheta = thetaRef - thetaFb; %rad
thetaDotMax = 0.5; %rad/sec
tInit = max(dTheta)/thetaDotMax;

dThetaPerSec = dTheta/tInit;
jointAngles= thetaFb;

tic;
while toc<tInit

    jointAngles = thetaFb+dThetaPerSec*toc;
    jointAngles(1) = NaN;

    cmd.position = jointAngles;
    cmd.torque = zeros(1,numModules);

    g.set(cmd);  

end

disp('Inititalization finished')
pause();

%% RUN
%%%%%%%%%%%%%%%%%%%
% Start recording %
%%%%%%%%%%%%%%%%%%%

tic; t= toc; count = 1;

% Clear structure in which the run log is saved
run_log = [];

while 1>0 

    %%% Stay still for 2 seconds
    t= max(0,double(toc)-2);
    
    %%% Load nominal helix shape (NB in USnake convention!!)
    params(1) = gaitSpeed * t;
    [xangles, yangles] = gaitInfo.gaitFunction(params, snakeData);
    jointAngles = nan(1,numModules);
    jointAngles(xDOF) = xangles(xDOF);
    jointAngles(yDOF) = yangles(yDOF);
    
    %%% Compute angles of the last three modules which rectify the head
    thetaTemporal = 2 * pi * gaitSpeed *t;
    if RECTIFY_HEAD 
        % Desired angle of the third last module, computed as evaluating the 
        % polynomial function at current point in time in the gait
        j0 = approximate( mod(thetaTemporal,2*pi), fit_theta0);
        % Desired angle for the second last module
        jm1 = approximate( mod(thetaTemporal,2*pi), fit_thetam1);
        % Desired angle for the last module (head)
        jm2 = approximate( mod(thetaTemporal,2*pi), fit_thetam2);  
        % Substitute the nominal rolling gait angles with the recitifed
        % ones
        jointAngles = jointAngles(4:end);
        jointAngles = [jm2 jm1 j0 jointAngles];        
    end
    
    %%% Transform in SEA snake convention
    jointAngles = anglesUtoSEA(snakeData, jointAngles) ;
    
    %%% Position control the snake
    % Limp the tail, so that the tether does not interfer with the gait
    jointAngles(1) = NaN;
    % Set no desired torque
    cmd.torque = zeros(1,numModules);
    cmd.position = jointAngles;
    g.set(cmd);

    %%% Get feedback
    fbk = g.getNextFeedback();
    
    if (~isempty(fbk))
            
        %%% Save in the log some useful information from the feedback
        run_log(count).t = fbk.time;    
        run_log(count).position = fbk.position;
        run_log(count).position_cmd = fbk.positionCmd;
        run_log(count).accel = [fbk.accelX; fbk.accelY;fbk.accelZ];
        run_log(count).gyro = [fbk.gyroX; fbk.gyroY;fbk.gyroZ];

        %%% Update the Complementary Filter pose estimate
        CF.update(fbk);
        
        %%% Save in the log the estimate of the Complementary Filter
        fk = CF.getInGravity ('wholeBody');
        VC = CF.getInGravity ('VC');
        head = CF.getInGravity ('head');
        VC_magnitudes = CF.getInGravity ('VC_magnitudes');
        run_log(count).head_frame = head;
        run_log(count).VC_frame = VC;
        run_log(count).all_frames = fk; 
        run_log(count).VC_magnitude = VC_magnitudes; 
        
    else
        disp('no fbk')
    end
    
    count = count+1;

end


%% SAVE THE RUN LOG
% THIS SENTENCE WILL NEVER BE REACHED
% IT IS JUST TO REMIND YOU TO SAVE THE RUN LOG
save('temp', 'run_log');
