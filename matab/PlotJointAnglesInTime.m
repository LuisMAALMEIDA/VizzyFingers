% Faz o plot do angulo das juntas no tempo

for j=1:3
    figure();
    
    JointIndex=j;
    TitleString = sprintf('Angulo da junta: %d', JointIndex);
    for i=1: length(tsJa.Time)
        subplot(3,1,1);
        plot(tsJa.Time(i), msgsJA{i,1}.JointAngles(JointIndex).X, 'rx');
        hold on;
        xlabel('Time');
        ylabel('Roll');
        
        subplot(3,1,2);
        plot(tsJa.Time(i), msgsJA{i,1}.JointAngles(JointIndex).Y, 'gx');
        hold on;
        xlabel('Time');
        ylabel('Pitch');

        subplot(3,1,3);
        plot(tsJa.Time(i), msgsJA{i,1}.JointAngles(JointIndex).Z, 'bx');
        hold on;
        xlabel('Time');
        ylabel('Yaw');

    end
    sgtitle(TitleString);
end