# Modelos YOLO

Coloque seu modelo treinado neste caminho (ou use caminho externo via parâmetro):

- `~/models/best.pt` (padrão da v1)

Também é possível passar no launch:

```bash
ros2 launch caatinga_vision ia_pipeline.launch.py model_path:=/caminho/para/best.pt
```

## Treino pelo Painel (Treino IA)

O painel `Caatinga Robotics - Central de Comando` possui a aba `Treino IA` para automatizar o fluxo:

1. `Subir Imagens` para `~/agro_robot_ws/datasets/agro_v1/raw/<session_id>/images`
2. `Subir Labels` para `~/agro_robot_ws/datasets/agro_v1/raw/<session_id>/labels`
3. `Organizar Split 80/10/10` para popular:
   - `images/train|val|test`
   - `labels/train|val|test`
4. `Teste de Contagem por Split` para validar pareamento imagem/label.
5. `Smoke Test (1 epoch)` para validar pipeline antes do treino completo.
6. `Treino Completo` com os parâmetros configurados na aba.

O treino usa `yolo detect train` via CLI e o arquivo:

- `~/agro_robot_ws/datasets/agro_v1/data.yaml`

## Coleta de Fotos em Rota (Coleta Fotos)

O painel também possui o implemento `📸 Coletor de Imagens (Treino)` e a aba `Coleta Fotos`.

Fluxo operacional:

1. Selecionar o implemento `📸 Coletor de Imagens (Treino)`.
2. Definir parâmetros de captura na aba `Coleta Fotos`.
3. Iniciar a rota com pendrive conectado (obrigatório).
4. As fotos são gravadas em:
   - `<usb>/Caatinga_Dados/Fotos_Treino/sess_<session_id>/images`
   - `<usb>/Caatinga_Dados/Fotos_Treino/sess_<session_id>/metadata_fotos.csv`
   - `<usb>/Caatinga_Dados/Fotos_Treino/sess_<session_id>/status_final.json`

Também é possível executar a captura por launch:

```bash
ros2 launch caatinga_vision photo_capture.launch.py usb_mount_path:=/media/$USER/SEU_USB session_id:=sess_teste
```
