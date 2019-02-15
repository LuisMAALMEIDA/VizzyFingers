% Calcula os coeficientes da reta que melhor se ajusta ao gráfico dos
% sensores e dos angulos da junta


Array_JA = linspace(min(JointAngle), max(JointAngle), 4000);
% Faz a simulação com o modelo calculado
% splineX = ppval(ppx,Array_JA);
% splineY = ppval(ppy,Array_JA);
% splineZ = ppval(ppz,Array_JA);

pX = polyfit(JointAngle,SensorArrayX,3);
pY = polyfit(JointAngle,SensorArrayY,3);
pZ = polyfit(JointAngle,SensorArrayZ,3);

% Identifica qual é o angulo das Juntas
Parameters(IndexFig).Joint = Xstring; 
Parameters(IndexFig).SensorZ = YstringZ;
Parameters(IndexFig).parX = pX;
Parameters(IndexFig).parY = pY;
Parameters(IndexFig).parZ = pZ;

splineX = polyval(pX,Array_JA);
splineY = polyval(pY,Array_JA);
splineZ = polyval(pZ,Array_JA);

subplot(3,1,1)
plot(Array_JA,splineX, 'y');

subplot(3,1,2)
plot(Array_JA,splineY, 'm');

subplot(3,1,3)
plot(Array_JA,splineZ, 'c' );
hold on;