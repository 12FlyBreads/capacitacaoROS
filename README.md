# capacitacaoROS
Projeto de capacitação em ROS2 da Black Bee Drones. O projeto consiste na simulação dos atributos de um drone (como potência dos motores, posição estimada e estado da missão), tudo isso utilizando o ROS2 e printando os valores no terminal.
---
## Funcionamento
O código criado foi algo bem mais simplificado do que a outra versão gráfica de simulação de drone, consistindo apenas numa pequena task para o drone realizar:
1. TAKEOFF no ponto (0,0,0) até 5 metros de altura;
2. MISSION = o drone se locomove em direção do eixo Z, e caso aviste um obstaculo ele deve desviar no eixo X e depois retornar a X=0 (essa parte foi bem simplificada, e o ponto mais levado em consideração  foi a utilização de ROS2);
3. LAND quando o drone chegar no destino (0,y,10);
4. IDLE quando for concluido e drone permanecer parado.
Essa simulação foi um pouco mais simplificada, generalizando a potencia dos motores na movimentação e não utilizando parametros de orientação (pitch, yaw, roll e throttle). O ponto principal focado nesse projeto foi a utilização do ROS2 na comunicação de nós.
   
## Tecnologias Utilizadas
- Python
- ROS2

## Estrutura do código
O código está na branch 'capacitacao', e é estruturado da seguinte forma:
├── launch
│   └── sim.launch.py // código para dar launch
├── sim_drone/
│   ├── __init__.py // inicialização obrigatória
│   ├── lidar_node.py // sensor lidar (nó que comunica com o drone)
│   ├── obstaculos.py // criação do obstaculo (para facilitar a simulação, funciona como um nó que comunica com o drone e o sensor)
│   └── drone_node.py // nó do drone (arquivo principal da simulação)
├── CMakeLists.txt
├── package.xml
├── setup.cfg
└── setup.py

## Como Rodar
```bash
# Comando launch
ros2 launch sim_drone sim.launch.py

