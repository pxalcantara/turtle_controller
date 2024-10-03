# Turtle Controller

O `turtle_controller` é um pacote desenvolvido para simplificar o controle de robôs diferenciais, sejam eles simulados ou reais. O objetivo principal é abstrair a complexidade dos comandos cinemáticos, permitindo que o usuário controle o robô utilizando comandos de alto nível, mais próximos de ações concretas. Dessa forma, elimina-se a necessidade de se preocupar com mensagens de baixa camada, como a `cmd_vel` do tipo `geometry_msgs/Twist`, facilitando o controle e a execução de movimentos.

Com o `turtle_controller`, comandos como a movimentação para frente com velocidade específica, ao invés de serem escritos manualmente com mensagens como:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Podem ser simplificados para um comando de alto nível, que especifica apenas a direção, a velocidade e a distância ou ângulo limite. Isso facilita a interação e integração com o robô, tornando o controle mais intuitivo e eficiente.

## Estrutura do Pacote
O `turtle_controller` utiliza uma interface simplificada de comandos através do tópico `/robot_cmd`, que recebe mensagens com o seguinte formato:

- __direction__ _(string)_: Especifica a direção do movimento. Pode assumir os seguintes valores:

    - `"front"`: Mover para frente.
    - `"back"`: Mover para trás.
    - `"left"`: Girar para a esquerda.
    - `"right"`: Girar para a direita.
    - `"stop"`: Parar o robô.

- __velocity__ _(float)_: Define a velocidade do movimento. Dependendo do valor de direction:

    - Para movimentos `front` e `back`, especifica a velocidade linear.
    - Para movimentos `left` e `right`, especifica a velocidade angular.

- __limit__ _(float)_: Define o limite de distância ou ângulo para o movimento:
    - Nos movimentos lineares (`front` e `back`), um valor positivo indica a distância máxima a ser percorrida. O valor `-1` indica que o robô deve se mover indefinidamente.
    - Nos movimentos angulares (`left` e `right`), especifica o ângulo de rotação em __radianos__.


## Requisito

Para funcionar corretamente, o `turtle_controller` necessita dos seguintes tópicos:

1. __/odom__ (`nav_msgs/Odometry`): Tópico que fornece a odometria do robô.
2. __/cmd_vel__ (`geometry_msgs/Twist`): Tópico que recebe comandos de velocidade para o controle do robô.

## Publicações

O `turtle_controller` publica informações sobre o status do robô no tópico `/robot_status`, utilizando a mensagem customizada `RobotStatus`, com o seguinte formato:

- **moving** (`bool`): Indica se o robô está se movendo (`true`) ou parado (`false`).
- **linear_velocity** (`float32`): Informa a velocidade linear atual do robô.
- **angular_velocity** (`float32`): Informa a velocidade angular atual do robô.
- **linear_distance** (`float32`): Indica a distância linear total percorrida pelo robô desde o início do movimento.
- **orientation** (`float32`): Orientação atual do robô em radianos, variando de 0 a 2π.
- **obstacle_distance** (`DirectionalDistance`): Mensagem com as distâncias para obstáculos nas direções ao redor do robô.

    - **front** (`float32`): Distância para um obstáculo à frente do robô.
    - **left** (`float32`): Distância para um obstáculo à esquerda do robô.
    - **right** (`float32`): Distância para um obstáculo à direita do robô.
    - **back** (`float32`): Distância para um obstáculo atrás do robô (essa medição não foi implementada ainda).
### Exemplo de Mensagem Publicada no Tópico `/robot_status`
```yaml
moving: true
linear_velocity: 0.5
angular_velocity: 0.0
linear_distance: 2.3
orientation: 1.57
obstacle_distance:
  front: 0.5
  left: 1.2
  right: 0.8
  back: 0.0  # Valor fixo indicando que a medição de distância para trás não foi implementada
```
## Aplicações

O turtle_controller é ideal para cenários onde a simplicidade e a clareza dos comandos são cruciais, como em simulações de comportamento, ensino de robótica e controle de robôs de uso geral. A interface simplificada facilita os primeiros contados com o mundo da robótica além de possibilitar o ensino de programação apenas, onde a robótica atua como papel motivador e contexto de aplicação das técnicas, mas não é o objetivo principal do aprendizado. 