# vizzy_fingers
Este _package_ ROS foi desenvolvido para se obterem os valores dos ângulos de rotação das juntas do dedo indicador do robot humanóide Vizzy. Para tal é necessário colocar várias _boards_ de marcadores _Aruco_ em cada uma das falanges do dedo como pode ser visto na imagem seguinte:

![alt text](https://raw.githubusercontent.com/LuisMAALMEIDA/VizzyFingers/master/imgs/markers.jpg "Board Makers on Vizzy's Finger")

Na pasta **imgs/boards** podem ser encontradas imagens das _boards_ para serem impressas. Depois de impressas, sugere-se a colagem de cada _board_ a um pedaço de cartão de forma a esta ficar o mais plana possível.
## Requisitos:
- Ter o OpenCV e o Opencontrib versão 3.3.1 instalado
- Ter o ROS instalado
- Ter uma câmera usb ligada ao PC

## Como inicializar:
- Fazer o download deste _package_ para dentro da pasta src do catkin_workspace (/catkin_ws/src)
- Na directoria (/catkin_ws), correr o comando: 

      catkin_make
- Calibrar a câmera para obter um ficheiro com os seus paramêtros
## Nós existentes:

- <h3>publish_board </h3>

    A posição e orientação das  _boards_  obtidas pela câmera são publicadas no tópico _Markers_chatter_. 
    O tipo de mensagens publicadas é um vetor de _markers_, em que cada _marker_ contém o identificador da _board_, a sua _pose_ e a transformada da _frame_ da _board_ em relação à _frame_ da câmera. 
    Além disto, as transformadas das _frames_ das _boards_ são publicadas no tópico _tf_.
    
    Argumentos:

    -  -c: directoria do ficheiro de calibração da câmera 
    
           ex: -c=/<install_dir>/catkin_ws/src/vizzy_fingers/calib_cam/camera_result.yml
    - --ci: Índice da câmera USB (câmera do PC: 0, outra câmera ligada ao PC: 1)
    - -d: Índice do dicionário usado
    - -w: Número de markers na posição X (_board_ width)
    - -h: Número de markers na posição Y (_board_ height)
    - -l: Tamanho do lado do marker (em cm)
    - -s: Separação entre dois markers consecutivos (em cm)
    - --rs: Aplicação da _refind strategy_
    
-  <h3>subscribe_tf </h3>

    Este nó subscreve as transformadas de cada _board_ em relação à _frame_ da câmera do tópico _tf_.  Assim, com estas transformadas, facilmente se obtêm as transformadas entre cada uma das _boards_ adjacentes (i.e. entre a 1ª e a 2ª, 2ª e a 3ª, 3ª e a 4ª _board_). Sendo que com  estas transformadas se conseguem obter os ângulos de rotação _roll_, _pitch_ e _yaw_. 
    
    Nota: Um ângulo de uma junta do dedo pode ser dado aproximadamente pelo ângulo _yaw_ da transformação entre as duas _boards_ que a rodeiam.

-  <h3>publish_markers </h3>

    A primeira tentativa para detetar o ângulo de rotação de cada uma das juntas do dedo, utilizou-se apenas um _marker_ em vez de uma _board_. 
    A vantagem da utilização das _boards_ em relação aos _markers_ deve-se ao facto de uma melhor precisão da _pose_ por parte das _boards_ relativamente aos _markers_. 
    Tirando esta diferença, a estrutura do _publish_markers_ é igual à do _publish_board_.
    
      Argumentos:

    -  -c: directoria do ficheiro de calibração da câmera 
    
           ex: -c=/<install_dir>/catkin_ws/src/vizzy_fingers/calib_cam/camera_result.yml
    - --ci: Índice da câmera USB (câmera do PC: 0, outra câmera ligada ao PC: 1)
    - -d: Índice do dicionário usado
    

-  <h3>subscribe_markers </h3>

      Subscreve um vetor de _markers_, imprimindo o seu identificador e a sua _pose_.
      Pode ser útil para fazer _debug_.
  
## Launches existentes:
