# caatingarobotics

Workspace ROS 2 com os pacotes:

- `src/agro_robot_sim`
- `src/caatinga_vision`

## Requisitos

- ROS 2 instalado
- `colcon`

## Build

```bash
cd /home/joaodemoura/agro_robot_ws
colcon build
```

## Execucao

Use os arquivos de launch de cada pacote conforme o fluxo desejado.

## Dataset e pesos

Este repositorio publica somente codigo e configuracoes essenciais.
Arquivos pesados (imagens, labels, pesos como `yolo11n.pt`) ficam fora do GitHub.

Se for treinar ou rodar inferencia, adicione localmente:

- dataset em `datasets/agro_v1/`
- pesos de modelo no caminho esperado pelos scripts/launch

