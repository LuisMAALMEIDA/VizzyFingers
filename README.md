# vizzy_fingers
Este package ROS foi desenvolvido para se obterem os valores dos ângulos de rotação das juntas do dedo indicador do robot humanóide Vizzy. Para tal é necessário colocar várias _boards_ de marcadores _Aruco_ em cada uma das falanges do dedo como pode ser visto na imagem seguinte:

![alt text](https://raw.githubusercontent.com/LuisMAALMEIDA/VizzyFingers/master/imgs/markers.jpg "Board Makers on Vizzy's Finger")

Na pasta **imgs/boards** podem ser encontradas imagens das _boards_ para serem impressas. Depois de impressas, sugere-se a colagem de cada _board_ a um pedaço de cartão de forma a esta ficar o mais plana possível.
## Requisitos:
- Ter o OpenCV e o Opencontrib versão 3.3.1 instalado
- Ter o ROS instalado
- Ter uma câmara usb ligada ao PC

## Como inicializar:
- Fazer o download deste package para dentro da pasta src do catkin_workspace (/catkin_ws/src)
- Na directoria (/catkin_ws), correr o comando: 

    catkin_make
- Calibrar a câmera para obter um ficheiro com os seus paramêtros
## Nós existentes:

- <h3>publish_board </h3>

    A posição e orientação das  _boards_  detetadas pelo 
    
    Argumentos:

    -  -c: directoria do ficheiro de calibração da câmara 
    ex: -c=/<install_dir>/catkin_ws/src/vizzy_fingers/calib_cam/camera_result.yml
    - --ci: Índice da câmara usb (câmara do PC: 0, outra câmara ligada ao PC: 1)
    - -d: Índice do dicionário usado
    - -w: Número de markers na posição X (_board_ weight)
    - -h: Número de markers na posição Y (_board_ height)
    - -l: Tamanho do lado do marker (em cm)
    - -s: Separação entre dois markers consecutivos (em cm)
    - --rs: Aplicação da _refind strategy_
- 
