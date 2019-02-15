% Atualiza as mensagens personalizadas do ROS
examplePackages = fullfile(fileparts(which('rosgenmsg')), 'examples', 'packages');
userFolder = '/home/luis/catkin_ws/src/vizzy_fingers/matab/msg';
copyfile(examplePackages, userFolder)
folderpath = userFolder;
rosgenmsg(folderpath)
%rosgenmsg("/home/luis/catkin_ws/src/");