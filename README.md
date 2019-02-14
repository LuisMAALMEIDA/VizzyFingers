# Vizzy fingers
Este package ROS foi desenvolvido para se obter os valores dos ângulos de rotação das juntas do dedo indicador do robot humanóide Vizzy. Para tal é necessário colocar várias _boards_ de marcadores _Aruco_ em cada uma das falanges do dedo como pode ser visto na imagem seguinte:

![alt text](https://raw.githubusercontent.com/LuisMAALMEIDA/VizzyFingers/master/imgs/markers.jpg "Board Makers on Vizzy's Finger")

Na pasta **imgs/boards** podem ser encontradas imagens das _boards_ para serem imprimidas. Depois de imprimidas, sugere-se a colagem de cada _board_ a um pedaço de cartão de forma a esta ficar o mais plana possível.
## Requisitos:
- Instalar o OpenCV e o Opencontrib versão 3.3.1
- Ter o ROS instalado
- Ter uma camara usb ligado ao pc

## Como inicializar:
- Fazer o download deste package para dentro da pasta src do catkin_workspace (/catkin_ws/src)
- Na directoria (/catkin_ws), correr o comando : 

    catkin_make

