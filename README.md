#### **IMPLEMENTAÇÃO DE ESTRATÉGIAS DE NAVEGAÇÃO EM ROBÓTICA AÉREA UTILIZANDO A PLATAFORMA AR.DRONE2.0**

----------
Neste trabalho se propõe testar técnicas para que um drone possa realizar trajetórias específicas e seguir alvos específicos de forma autônoma, com implementação em uma plataforma experimental.

Inicialmente se espera que o drone siga trajetórias fechadas, como um circulo, por meio de decomposição em campos vetoriais. O segundo objetivo é que o drone possa detectar por meio de visão computacional um alvo e o siga com uma velocidade mínima especificada. Finalmente deseja-se integrar as duas soluções de modo que, quando o drone não detectar ou perder o alvo, fique executando uma trajetórias especificada até detectar um novo alvo. Almeja-se também publicar um artigos em algum congresso nacional.

O sistema de simulação de dispositivos Robóticos Gazebo será usado na primeira etapa do projeto, após sucesso na simulação a aplicação será implementada em uma plataforma experimental. 

----------
## Run App
[Intall ros-indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

### Compile
``` shell 
$ . deve/setup.bash
$ catkin_make
```
### Run AR.drone
``` shell 
$ roslaunch ardrone_autonomy ardrone.launch
$ roslaunch roslaunch tum_ardrone tum_ardrone.launch
$ rosrun drone_cv drone_cv_node
# or put a target
$ rosrun drone drone_node
```
### Run Simulate
``` shell 
$ roslaunch roslaunch cvg_sim_gazebo ardrone_testworld.launch
$ rosrun drone_cv drone_cv_node
# or 
$ roslaunch ardrone_tutorials keyboard_controller_simu_empty.launch 
# put a target
$ rosrun drone drone_node
```

