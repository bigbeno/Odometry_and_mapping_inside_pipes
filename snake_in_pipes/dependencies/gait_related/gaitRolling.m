function [ gaitInfo ] = gaitRolling( )
%GAITROLLING Returns a struct with information about the gait, along with
%the function handle to the gait equation that generates joint angles.
%
%   [ gaitInfo ] = gaitRolling( )
%
%   gaitInfo
%       .gaitName - A string of the name of the gait.
%
%       .gaitParams - A cell array of the names of the gait parameters, in
%                     the order in which they're used by the gait function.
%
%       .gaitUnits - A cell array of the units of the gait parameters, in
%                    order corresponding to gait parameters.
%
%       .defaultParams - An array of the default gait parameters.
%
%       .paramNoise - An array of noise values, used as the additive noise
%                     values in EKF / UKF state estimators.  There will be
%                     (2 x numParams) noise values, since they are for all
%                     the gait parameters and their 1st derivatives.
%
%       .axesVC - The 3x3 permutation matrix defining the x-y-z axes of the
%                 Virtual Chassis.
%
%       .numParams - The number of gait parameters.
%
%       .gaitFunction - A handle to the actual function that generates
%                       joint angles.
%
%   More stuff could be added later, that's why its a struct!
%
%   Dave Rollinson
%   Apr 2012

    %%%%%%%%%%%%%%%%%%%%
    % Gait Information %
    %%%%%%%%%%%%%%%%%%%%
    
    % Gait Name
    gaitInfo.gaitName = 'rollingBoth';
    
    % Gait Parameters 
    % These should match their usage in the gait equation (below).
    gaitInfo.gaitParams = { 'position';
                            'amplitude';
                            'spatial offset'};
    
    % Gait Parameter Units 
    % These should match their usage in the gait equation (below). 
    gaitInfo.gaitUnits = { 'cycles';
                           'radians';
                           'radians'};  
    
    % Default Gait Parameters 
    gaitInfo.defaultParams = [ 0; 
                               .4;
                               0];
                           
    % Parameter noise values.  These are basically 'tune-it-til-it-works'
    % parameters, based on running the state estimation code.
    %
    % Due to the way noise gets incorportated in the process model in
    % snakeEKF, only put noises on the derivatives.  The filter will
    % integrate the first derivatives appropriately.
    gaitInfo.paramNoise = [ 
                           0        % gait position
                           0.01        % gait speed
                           0        % amplitude
                           0.01        % d_amp / dt
                       ];
                   
    % Axes for the Virtual Chassis
    % Default is identity matrix, so x-y-z are 1st-2nd-3rd moments
    gaitInfo.axesVC = eye(3);   
    
    % Total number of parameters
    gaitInfo.numParams = length(gaitInfo.gaitParams);
    
    % A handle to the gait equation (below).  It is called gaitEquation
    % primarily because it had to be something different than gaitFunction.
    gaitInfo.gaitFunction = @gaitFunction;
end


%%%%%%%%%%%%%%%%%
% Gait Equation %
%%%%%%%%%%%%%%%%%

function [ xAngles, yAngles ] = gaitFunction( gaitParams, snakeData )
%GAITFUNCTION Returns a set of module angles given gait parameters.
%   Forms an arc of constant curvature, and rolls through that arc.

    % Vector of module numbers
    modules = 1:snakeData.num_modules;

    pos = gaitParams(1);    % Position
    amp = gaitParams(2);    % Amplitude
    tauPerM = gaitParams(3);% tau*m (m= module length)

    % Gait Equation
    xAngles = ...
            amp * sin( pos*(2*pi) + tauPerM*modules);
    yAngles = ...
            amp * cos( pos*(2*pi) + tauPerM*modules );
    
end
