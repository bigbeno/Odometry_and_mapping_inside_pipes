%% Read and Set Gains
% Iterate thru all the groups and set the gains to default

for i=1:1 %HebiApi.getNumGroups
    
    gainGroup = HebiLookup.newConnectedGroupFromName('*','SA002');
    
    gains = gainGroup.getGains();

    numModules = gainGroup.getNumModules;

    % Maximum velocity of SEA joints is 33 rpm; we convert to rad/s here.
    maxVelocityRadS = (33 * 2 * pi) / 60;

    ones_n = ones(1,numModules);
    
    gains.time = g.getNextFeedback().time;
    gains.controlStrategy = ones_n*4;
    
    gains.positionKp = ones_n*10;
    gains.positionKi = ones_n*0;
    gains.positionKd = ones_n*0;
    gains.positionFF = ones_n*0;
    gains.positionDeadZone = ones_n*0;
    gains.positionIClamp = ones_n*0;
    gains.positionPunch = ones_n*0;

    gains.positionMinTarget = ones_n*-1.7708;
    gains.positionMaxTarget = ones_n*1.7708;
    gains.positionMinOutput = ones_n*-12;
    gains.positionMaxOutput = ones_n*12;
    
    gains.positionTargetLowpassGain = ones_n*1;
    gains.positionOutputLowpassGain = ones_n*1;
    gains.positionDOnError = ones_n*1;
    
    gains.velocityKp = ones_n*0.1000;
    gains.velocityKi = ones_n*1.0e-04 *1;
    gains.velocityKd = ones_n*0;
    gains.velocityFF = ones_n*0.3086;
    gains.velocityDeadZone = ones_n*0.0100;
    gains.velocityIClamp = ones_n*0.2000;
    gains.velocityPunch = ones_n*0;
    
    gains.velocityMinTarget = ones_n*-4.0508 ;
    gains.velocityMaxTarget = ones_n*4.0508;
    gains.velocityMinOutput = ones_n*1;
    gains.velocityMaxOutput = ones_n*1;
    
    gains.velocityTargetLowpassGain = ones_n*1;
    gains.velocityOutputLowpassGain = ones_n*0.5;
    gains.velocityDOnError = ones_n*1;
    
    gains.torqueKp = ones_n*0.7500;
    gains.torqueKi = ones_n*0;
    gains.torqueKd = ones_n*1;
    gains.torqueFF = ones_n*0.1488 ;
    gains.torqueDeadZone = ones_n*0.0100 ;
    gains.torqueIClamp = ones_n*0;
    gains.torquePunch = ones_n*0;
    
    gains.torqueMinTarget = ones_n*-12;
    gains.torqueMaxTarget = ones_n*12;
    gains.torqueMinOutput = ones_n*-1;
    gains.torqueMaxOutput = ones_n*1;
    
    gains.torqueTargetLowpassGain = ones_n*1;
    gains.torqueOutputLowpassGain = ones_n*0.1500;
    gains.torqueDOnError = ones_n*0;
    

    % Set the Gains (NO PERSIST)
    gainGroup.set('gains', gains, 'led', 'y');
    
    % Set the Gains (PERSIST)
%     gainGroup.set('gains', gains, 'persist', true, 'led', 'y');

end

pause(2.0);

for i=1:1 %HebiApi.getNumGroups
    
    gainGroup =HebiLookup.newConnectedGroupFromName('*','SA002');
    
    % Set the Gains (PERSIST)
    gainGroup.set('led', []);
end
