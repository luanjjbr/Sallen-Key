# Projeto Final - Controle 2 (Planta Sallen-Key)

Códigos e dados para o trabalho final de Controle 2. Contém a extração de dados de uma planta Sallen-Key via Arduino, a identificação do sistema no MATLAB e o projeto do controlador.

## 🛠️ Fases do Projeto

### 1. Coleta de Dados (Arduino)

- **Hardware:** Placa microcontroladora Arduino Nano (disponibilizada pelo professor).
- **Firmware:** Código base adaptado do projeto [Sallen-Key-DuinoV2_PID](https://oshwlab.com/daniel_bezerra/sallen-key-duino) no OSHWLab.
- **Estímulo:** Entrada do tipo degrau.
- **Processamento dos Dados:** A coleta foi realizada via extensão **Teleplot** no VS Code. O log gerado foi processado por um script em Python (`grafico.ipynb`) para conversão em formato `.csv` e posterior análise no MATLAB.
  - _Nota:_ Foi feita uma paginação/limpeza manual no arquivo CSV para remover os valores nulos (zeros) anteriores à aplicação do degrau.

### 2. Identificação do Sistema (Função de Transferência)

Com os dados práticos normalizados, os parâmetros do sistema de segunda ordem foram calculados através das equações de sobressinal e regime permanente:

- **Sobressinal Máximo ($M_p$):**
  $$M_p = \frac{y_{pico} - y_{estabilizado}}{y_{estabilizado}} = \frac{578 - 329}{329} = 0,7568 \text{ (75,68\% de overshoot)}$$

- **Fator de Amortecimento ($\xi$):**
  $$\xi = \frac{- \ln(M_p)}{\sqrt{\pi^2 + \ln^2(M_p)}} = \frac{- \ln(0,7568)}{\sqrt{\pi^2 + \ln^2(0,7568)}} = 0,088$$

- **Ganho Estático ($K$):**
  - _Análise para degrau de amplitude 353:_ $K = \frac{y_{estabilizado}}{u} = \frac{329}{353} = 0,932$
  - _Análise para degrau de amplitude 512:_ $K = \frac{y_{estabilizado}}{u} = \frac{329}{512} = 0,642$

- **Função de Transferência Contínua $G(s)$:**
  $$G(s) = \frac{98.88}{s^2 + 1.751s + 106.1}$$

- **Função de Transferência Discreta $G(z)$ (ZOH, $T_s = 10\text{ ms}$):**
  $$G(z) = \frac{0.004911z + 0.004882}{z^2 - 1.972z + 0.9826}$$
