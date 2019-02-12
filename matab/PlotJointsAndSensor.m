%UNTITLED Summary of this function goes here
%   Faz o plot dos angulos da junta JointIndex e do sensor SensorIndex

YstringX = sprintf('Sensor %d X [Oe]', SensorIndex-1);
YstringY = sprintf('Sensor %d Y [Oe]', SensorIndex-1);
YstringZ = sprintf('Sensor %d Z [Oe]', SensorIndex-1);

% Desenha os gráficos tendo em conta que o angulo de rotaçao é o Roll
if VectJointAngle(1) == 1
    IndexFig=IndexFig+1;
    h(IndexFig)=figure();
    TitleString = sprintf('Roll %d ª Joint and the %dº Tactile Sensor', JointIndex, SensorIndex-1);
    Xstring = sprintf('Joint angle %d x [rad]', JointIndex);
    
    % Limpa os arrays
    CleanArrays;
    % Plot para o angulo da primeira junta(roll) e do sensor 5
    for i = 1: length(tsJa.Time)
        
        % Guarda os dados dos vetores 
        JointAngle(i) = Dados{i,1}.JointAngles(JointIndex).X;
        SensorArrayX(i)= Dados{i,2}.SensorArray(SensorIndex).Field.X;
        SensorArrayY(i) = Dados{i,2}.SensorArray(SensorIndex).Field.Y;
        SensorArrayZ(i)= Dados{i,2}.SensorArray(SensorIndex).Field.Z;
        
        
        p(1) = subplot(3,1,1)
        plot(Dados{i,1}.JointAngles(JointIndex).X, Dados{i,2}.SensorArray(SensorIndex).Field.X, 'rx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringX);
        p(2) = subplot(3,1,2)
        plot(Dados{i,1}.JointAngles(JointIndex).X, Dados{i,2}.SensorArray(SensorIndex).Field.Y, 'gx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringY);
        p(3) = subplot(3,1,3)
        plot(Dados{i,1}.JointAngles(JointIndex).X, Dados{i,2}.SensorArray(SensorIndex).Field.Z, 'bx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringZ);

    end
    Plotspline;
    sgtitle(TitleString);
    
end

% Desenha os gráficos tendo em conta que o angulo de rotaçao é o Pitch
if VectJointAngle(2) == 1
    IndexFig=IndexFig+1;
    h(IndexFig)=figure();
    TitleString = sprintf('Pitch %d ª Joint and the %dº Tactile Sensor', JointIndex, SensorIndex-1);
    Xstring = sprintf('Joint angle %d y [rad]', JointIndex);
    
    % Limpa os arrays
    CleanArrays;
    % Plot para o angulo da primeira junta(roll) e do sensor 5
    for i = 1: length(tsJa.Time)

        % Guarda os dados dos vetores 
        JointAngle(i) = Dados{i,1}.JointAngles(JointIndex).Y;
        SensorArrayX(i)= Dados{i,2}.SensorArray(SensorIndex).Field.X;
        SensorArrayY(i) = Dados{i,2}.SensorArray(SensorIndex).Field.Y;
        SensorArrayZ(i)= Dados{i,2}.SensorArray(SensorIndex).Field.Z;
        
        subplot(3,1,1)
        plot(Dados{i,1}.JointAngles(JointIndex).Y, Dados{i,2}.SensorArray(SensorIndex).Field.X, 'rx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringX);
        subplot(3,1,2)
        plot(Dados{i,1}.JointAngles(JointIndex).Y, Dados{i,2}.SensorArray(SensorIndex).Field.Y, 'gx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringY);
        subplot(3,1,3)
        plot(Dados{i,1}.JointAngles(JointIndex).Y, Dados{i,2}.SensorArray(SensorIndex).Field.Z, 'bx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringZ);

    end
    Plotspline;
    sgtitle(TitleString);
end 
 % Desenha os gráficos tendo em conta que o angulo de rotaçao é o Yaw
if VectJointAngle(3) == 1
    IndexFig=IndexFig+1;
    h(IndexFig)=figure();
    Xstring = sprintf('Joint angle %d z [rad]', JointIndex);
    TitleString = sprintf('Yaw %d ª Joint and the %dº Tactile Sensor', JointIndex, SensorIndex-1);
    
    % Limpa os arrays
    CleanArrays;
    % Plot para o angulo da primeira junta(roll) e do sensor 5
    for i = 1: length(tsJa.Time)

        % Guarda os dados dos vetores 
        JointAngle(i) = Dados{i,1}.JointAngles(JointIndex).Z;
        SensorArrayX(i)= Dados{i,2}.SensorArray(SensorIndex).Field.X;
        SensorArrayY(i) = Dados{i,2}.SensorArray(SensorIndex).Field.Y;
        SensorArrayZ(i)= Dados{i,2}.SensorArray(SensorIndex).Field.Z;
        
        subplot(3,1,1)
        plot(Dados{i,1}.JointAngles(JointIndex).Z, Dados{i,2}.SensorArray(SensorIndex).Field.X, 'rx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringX);
        subplot(3,1,2)
        plot(Dados{i,1}.JointAngles(JointIndex).Z, Dados{i,2}.SensorArray(SensorIndex).Field.Y, 'gx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringY);
        subplot(3,1,3)
        plot(Dados{i,1}.JointAngles(JointIndex).Z, Dados{i,2}.SensorArray(SensorIndex).Field.Z, 'bx');
        hold on;
        xlabel(Xstring);
        ylabel(YstringZ);

    end
    Plotspline;
    sgtitle(TitleString);
end    


