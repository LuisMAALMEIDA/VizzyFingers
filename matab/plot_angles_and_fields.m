% Script to plots the angle joint's and the magnetic sensor field's
clc; close all; clear;
%bag = rosbag('/home/luis/catkin_ws/src/vizzy_fingers/rosbags/2018-11-29-10-54-05.bag');
bag = rosbag('/home/luis/catkin_ws/src/vizzy_fingers/rosbags/2018-11-30-11-20-27.bag');



% Bag Joint Angles
bagsJA = select(bag, 'Topic', 'joint_angles_fingers_topic');
% Bag with Tactile sensors measures
bagsTactile = select( bag, 'Topic' , 'tactileForceField');

% Message Joint Angles
msgsJA = readMessages(bagsJA,'DataFormat','struct');
msgsTactile= readMessages(bagsTactile , 'DataFormat', 'struct');

%Time series
[tsJa, colsJa] = timeseries(bagsJA);
[tsTactile, colsTactile]=timeseries(bagsTactile);


% Obtem-se a rotação em cada uma das juntas em relação ao referecial da mao
for i=1:length(msgsJA)
    
    % Soma do agulo 1 e 2 -> no indice 2
    msgsJA{i,1}.JointAngles(2).X = msgsJA{i,1}.JointAngles(2).X + msgsJA{i,1}.JointAngles(1).X;
    msgsJA{i,1}.JointAngles(2).Y = msgsJA{i,1}.JointAngles(2).Y + msgsJA{i,1}.JointAngles(1).Y;
    msgsJA{i,1}.JointAngles(2).Z = msgsJA{i,1}.JointAngles(2).Z + msgsJA{i,1}.JointAngles(1).Z;
    
    % Soma do angulo 1, 2 e 3 -> no indice 3
    msgsJA{i,1}.JointAngles(3).X = msgsJA{i,1}.JointAngles(3).X + msgsJA{i,1}.JointAngles(2).X;
    msgsJA{i,1}.JointAngles(3).Y = msgsJA{i,1}.JointAngles(3).Y + msgsJA{i,1}.JointAngles(2).Y;
    msgsJA{i,1}.JointAngles(3).Z = msgsJA{i,1}.JointAngles(3).Z + msgsJA{i,1}.JointAngles(2).Z;
    
end

% Vamos obter os dados das Juntas em função dos sensores tacteis
% Pré-alocando memoria (contem dados dos angulos e dos sensores tacteis)

Dados = [msgsJA msgsJA];
Time = [ tsJa.Time tsJa.Time];

for i=1:length(tsJa.Time)
    
    % Encontra o index do tempo que está mais próximo do angulo da junta
    [~ , index] = min(abs(tsTactile.Time-tsJa.Time(i)));
    % Guarda os dados do sensor para esse index
    Dados{i,2}= msgsTactile{index,1};
    % Guarda o tempo em que os dados do sensor foram guardados para esse index
    Time(i,2) = tsTactile.Time(index);
    disp(i)
    disp(index)
    disp('----------------------------------')
    
end



%% Plots

% Faz o plot de todas as combinações entre sensores e o angulo das juntas
VecSensor = [3 5 4 6]+1; % No matlab os indices começam em 1, no Ros começa em 0
VectJointAngle=[0 0 1];
IndexFig=0;
for k= 1:3
    JointIndex=k;
    
    for j=1:4
        waitbar(((k-1)*3+j)/12)
        SensorIndex= VecSensor(j);
        PlotJointsAndSensor;
        
    end
end

savefig(h,'imgs/TwoFiguresFile1.fig');
%figs = openfig('TwoFiguresFile.fig'); %Open figs
%spline
%% Plot dos angulos no tempo
%PlotJointAnglesInTime


