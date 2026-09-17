# 🎛️ Projeto de Controladores Digitais — Planta Sallen-Key-Duino

> **Trabalho Final da Disciplina de Controle de Sistemas Amostrados (Controle II)**  
> **Instituto Federal de Educação, Ciência e Tecnologia do Ceará (IFCE) — Campus Maracanaú**  
> **Curso de Engenharia de Controle e Automação**  
> **Autor:** Luan Italo Mota Sousa  
> **Orientação/Hardware Base:** Prof. Daniel Bezerra ([Sallen-Key-Duino no OSHWLab](https://oshwlab.com/daniel_bezerra/sallen-key-duino))

---

## 📌 Sumário
1. [Visão Geral do Projeto](#-visão-geral-do-projeto)
2. [Hardware e Setup Experimental](#-hardware-e-setup-experimental)
3. [Modelagem e Identificação Paramétrica](#-modelagem-e-identificação-paramétrica)
4. [Análise em Malha Aberta (Sem Controle)](#-análise-em-malha-aberta-sem-controle)
5. [Projeto e Comparação dos Controladores](#-projeto-e-comparação-dos-controladores)
6. [Firmware Arduino e Controle em Tempo Real](#-firmware-arduino-e-controle-em-tempo-real)
7. [Simulações e Validação no MATLAB](#-simulações-e-validação-no-matlab)
8. [Estrutura do Repositório](#-estrutura-do-repositório)
9. [Como Reproduzir o Projeto](#-como-reproduzir-o-projeto)
10. [Referências Bibliográficas](#-referências-bibliográficas)

---

## 📖 Visão Geral do Projeto

Este projeto documenta o ciclo completo de engenharia de controle discreto aplicado a uma planta física real: a placa didática **Sallen-Key-Duino**, um circuito eletrônico analógico ativo de 2ª ordem subamortecido.

O fluxo de desenvolvimento contempla:
1. **Aquisição de Dados:** Extração do sinal temporal da planta via conversor analógico-digital de um microcontrolador Arduino Nano sob estímulo degrau;
2. **Identificação de Sistemas:** Estimação dos parâmetros do modelo contínuo $G(s)$ ($K$, $\xi$, $\omega_n$) e obtenção da função de transferência em tempo discreto $G(z)$ via Segurador de Ordem Zero (ZOH);
3. **Projeto no Domínio Z e Frequência:** Projeto analítico e computacional de 6 estratégias de controle (P, Avanço de Fase, PI, Avanço+PI em cascata, PID Robusto e Dead-Beat de Dahlin);
4. **Implementação Embarcada:** Programação em C++ das respectivas equações de diferenças com proteção contra saturação (*anti-windup*) e temporização determinística de $10\text{ ms}$;
5. **Validação Experimental:** Confronto das respostas planejadas no MATLAB com os dados reais de telemetria medidos na bancada.

---

## ⚡ Hardware e Setup Experimental

* **Microcontrolador:** Arduino Nano (ATmega328P, 16 MHz).
* **Planta Analógica:** Filtro ativo passa-baixas Sallen-Key de 2ª ordem implementado com amplificadores operacionais.
* **Geração de Tensão Simétrica Negativa (-5 Vcc):**
  * O `Timer1` do microcontrolador foi configurado com prescaler 1 para gerar PWM em **31.372,55 Hz** (inaudível e de fácil filtragem).
  * O pino digital `D10` gera uma onda quadrada de 50% de duty cycle para chavear o circuito de bomba de carga gerador de $-5\text{ Vcc}$.
* **Atuação (Entrada do Sistema):** Sinal PWM no pino `D9` (`pwm_PIN_Vin_SallenKey`), variando de 0 a 255.
* **Sensoriamento (Saídas):**
  * Pino `A6` ($V_o$): Tensão na saída do filtro Sallen-Key;
  * Pino `A5` ($V_g$): Tensão intermediária no circuito;
  * Pino `A7` ($V_{gK}$): Tensão de ganho.
* **Período de Amostragem ($T_s$):**
  * $T_s = 10\text{ ms}$ ($f_s = 100\text{ Hz}$), temporizado por laço determinístico não-bloqueante com `millis()`.
  * O pino digital `D12` atua como monitor de debug: mantido em nível alto durante a execução do laço de controle, permitindo verificar a folga de processamento em osciloscópio.
* **Aquisição e Telemetria:** Transmissão serial a 115.200 bps e captura pelo plugin **Teleplot** do VS Code.

---

## 📐 Modelagem e Identificação Paramétrica

A planta Sallen-Key é modelada pela função de transferência canônica de segunda ordem:

$$G(s) = \frac{K \cdot \omega_n^2}{s^2 + 2\xi\omega_n s + \omega_n^2}$$

### Parâmetros Extraídos do Ensaio ao Degrau (Entrada = 500):
A partir do ensaio prático em malha aberta registrado em [`matlab/dados/Vo.csv`](matlab/dados/Vo.csv):

* **Ganho DC ($K$):**
  $$K = \frac{y_{\text{estabilizado}}}{u_{\text{degrau}}} = \frac{326}{500} = 0,652$$
* **Sobressinal Máximo ($M_p$):**
  $$M_p = \frac{y_{\text{pico}} - y_{\text{estabilizado}}}{y_{\text{estabilizado}}} = \frac{578 - 327}{327} = 0,7676 \quad (76,76\%)$$
* **Fator de Amortecimento ($\xi$):**
  $$\xi = \frac{-\ln(M_p)}{\sqrt{\pi^2 + \ln^2(M_p)}} = \frac{-\ln(0,7676)}{\sqrt{\pi^2 + \ln^2(0,7676)}} \approx 0,0839$$
* **Frequência Natural ($\omega_n$):**
  Medindo o período da oscilação amortecida ($N = 61$ amostras a $T_s = 0,01\text{ s} \implies T_d = 0,61\text{ s}$):
  $$\omega_n \approx \omega_d = \frac{2\pi}{N \cdot T_s} = \frac{2\pi}{61 \times 0,01} \approx 10,30\text{ rad/s}$$


### Funções de Transferência Resultantes:

* **Domínio Contínuo $G(s)$:**
  $$G(s) = \frac{0,652 \cdot (10,30)^2}{s^2 + 2(0,0839)(10,30)s + (10,30)^2} = \frac{69,17}{s^2 + 1,728 s + 106,10}$$

* **Domínio Discreto $G(z)$ (ZOH com $T_s = 10\text{ ms}$):**

  $$G(z) = \mathcal{Z}\left\{\frac{1-e^{-sT_s}}{s}G(s)\right\} = \frac{0,003436 z + 0,003416}{z^2 - 1,972 z + 0,9829}$$


* **Polos em Malha Aberta:**
  $$z_{1,2} = 0,9860 \pm j 0,1028 \quad (|z| = 0,9914)$$
  *Polos conjugados complexos muito próximos da fronteira do círculo unitário, justificando a oscilação acentuada e a baixa estabilidade relativa sem controle.*

---

## ⚠️ Análise em Malha Aberta (Sem Controle)

A planta em malha aberta apresenta características críticas que inviabilizam seu uso sem compensação:
* **Sobressinal:** $76,70\%$
* **Tempo de Acomodação ($t_s$ a $2\%$):** $4,35\text{ s}$
* **Erro em Regime Permanente ($e_{ss}$):** $34,80\%$
* **Margem de Ganho (MG):** $14,01\text{ dB}$
* **Margem de Fase (MF):** $15,36^\circ$

---

## 🎯 Projeto e Comparação dos Controladores

Foram projetados, simulados no MATLAB e embarcados no Arduino seis estratégias de controle digital:

### 1. Tabela Comparativa de Desempenho

| Controlador | Tempo de Subida ($t_r$) | Acomodação ($t_s$) | Sobressinal ($M_p$) | Ganho DC | Erro Regime ($e_{ss}$) | Margem de Ganho (MG) | Margem de Fase (MF) | Avaliação de Engenharia |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Sem Controle (SC)** | $0,11\text{ s}$ | $4,35\text{ s}$ | $76,70\%$ | $0,652$ | $34,80\%$ | $14,01\text{ dB}$ | $15,36^\circ$ | Resposta natural altamente oscilatória e imprecisa. |
| **Proporcional (P)** | $0,07\text{ s}$ | $6,51\text{ s}$ | $87,72\%$ | $0,499$ | $50,06\%$ | $10,31\text{ dB}$ | $9,52^\circ$ | **Ineficaz:** piora o sobressinal e degrada as margens de estabilidade. |
| **Avanço de Fase (AA)** | $\mathbf{0,01\text{ s}}$ | $\mathbf{0,16\text{ s}}$ | $26,89\%$ | $0,888$ | $11,20\%$ | $11,67\text{ dB}$ | $58,68^\circ$ | **Ultra rápido**, melhora amortecimento mas mantém erro residual. |
| **Proporcional-Integral (PI)** | $1,91\text{ s}$ | $5,34\text{ s}$ | $0,86\%$ | $\mathbf{1,000}$ | $\mathbf{\approx 0\%}$ | $6,17\text{ dB}$ | $96,05^\circ$ | Elimina erro em regime permanente, mas transitório é lento. |
| **Avanço + PI (API)** | $1,56\text{ s}$ | $3,00\text{ s}$ | $\mathbf{0\%}$ | $\mathbf{1,000}$ | $\mathbf{\approx 0\%}$ | $33,87\text{ dB}$ | $39,34^\circ$ | Cascata de 2ª ordem aliando velocidade do avanço com erro nulo. |
| **PID Robusto (PID)** ⭐ | $1,72\text{ s}$ | $\mathbf{2,96\text{ s}}$ | $\mathbf{0\%}$ | $\mathbf{1,000}$ | $\mathbf{\approx 0\%}$ | $\mathbf{40,50\text{ dB}}$ | $\mathbf{97,32^\circ}$ | **Campeão Global:** sobressinal zero, erro nulo e máximas margens. |

---

### 2. Equações de Diferenças Implementadas

#### A. Controlador Proporcional (P)
* $K_p = 1,53$
* **Equação de Diferenças:**
  $$u[k] = 1,53 \cdot e[k]$$

#### B. Compensador de Avanço de Fase (AA)
* Alocação: Zero em $z_c = 0,85$, Polo em $p_c = -0,85$, Ganho $K_c = 150$.
* Função de Transferência:
  $$D(z) = 150 \frac{z - 0,85}{z + 0,85}$$
* **Equação de Diferenças:**
  $$u[k] = -0,85 \cdot u[k-1] + 150 \cdot e[k] - 127,5 \cdot e[k-1]$$

#### C. Controlador Proporcional-Integral (PI)
* Alocação: $z_c = 0,925$, $K_c = 0,20 \implies K_p = 0,20$, $K_i = 1,50$.
* **Equação de Diferenças:**
  $$I[k] = \text{clip}(I[k-1] + K_i \cdot T_s \cdot e[k], -1.0, 1.0)$$
  $$u[k] = K_p \cdot e[k] + I[k]$$

#### D. Compensador Avanço + PI em Cascata (API)
* Associação do compensador de avanço ($z_{c1} = 0,90, p_{c1} = 0,30, K_{c1} = 2$) com o PI ($z_{c2} = 0,95, K_{c2} = 1,90$).
* Função de Transferência de 2ª Ordem:
  $$C(z) = \frac{3,800 z^2 - 7,030 z + 3,249}{z^2 - 1,300 z + 0,300}$$
* **Equação de Diferenças:**
  $$u[k] = 1,300 \cdot u[k-1] - 0,300 \cdot u[k-2] + 3,800 \cdot e[k] - 7,030 \cdot e[k-1] + 3,249 \cdot e[k-2]$$

#### E. Controlador PID Robusto (PID) ⭐
* Ganhos calculados: $K_p = 0,25$, $K_i = 2,25$, $K_d = 0,025$.
* Discretização paralela:
  $$u[k] = K_p \cdot e[k] + \sum (K_i \cdot T_s \cdot e[k]) + \frac{K_d}{T_s} (e[k] - e[k-1])$$
* *Resultado:* A ação derivativa injeta o amortecimento ausente na planta natural, permitindo eliminar o sobressinal ($M_p = 0\%$) com tempo de estabilização em $2,96\text{ s}$.

#### F. Controlador Dead-Beat (Dahlin - DB)
* Projetado com tempo de acomodação desejado de $t_s \approx 6\text{ s}$ para evitar a saturação excessiva do PWM do conversor.
* **Equação de Diferenças:**
  $$u[k] = 0,00575 \cdot u[k-1] + 0,99425 \cdot u[k-2] + 1,93387 \cdot e[k] - 3,81427 \cdot e[k-1] + 1,90073 \cdot e[k-2]$$

---

## 💻 Firmware Arduino e Controle em Tempo Real

O firmware está localizado em [`firmware/Sallen-Key-DuinoV2_PID/Sallen-Key-DuinoV2_PID.ino`](firmware/Sallen-Key-DuinoV2_PID/Sallen-Key-DuinoV2_PID.ino).

### Recursos Implementados:
* **Chaveamento Dinâmico em Execução:** Permite enviar comandos via Serial Monitor para alternar o controlador ativo instantaneamente:
  * `SC` $\rightarrow$ Sem Controle (Malha aberta)
  * `P` $\rightarrow$ Proporcional
  * `PI` $\rightarrow$ Proporcional + Integral
  * `PID` $\rightarrow$ PID Robusto
  * `AA` $\rightarrow$ Avanço de Fase
  * `API` $\rightarrow$ Avanço + PI
  * `DB` $\rightarrow$ Dead-Beat (Dahlin)
* **Zeração de Estados:** Ao trocar de modo, a função `resetStates()` zera acumuladores e memórias de erro para evitar solavancos transitórios.
* **Anti-Windup:** Proteção integral implementada com saturação rígida em $[-1.0, 1.0]$.
* **Saturação de Saída:** A variável calculada $u(k) \in [0.0, 1.0]$ é mapeada para o PWM do pino D9 em $[0, 255]$ através de `constrain()`.

---

## 🔬 Simulações e Validação no MATLAB

Os scripts de projeto e simulação estão concentrados no diretório [`matlab/`](matlab/):

* **[`matlab/teste3.m`](matlab/teste3.m) (Script Principal de Validação):**
  * Localiza os arquivos `.csv` automaticamente através da pasta [`matlab/dados/`](matlab/dados/);
  * Simula a resposta ao degrau teórica em malha fechada via `step(ref * MF)`;
  * Sincroniza a curva teórica com a resposta gravada do Arduino Nano;
  * Gera gráficos individuais de validação (*Planejado vs Medido*) e o gráfico consolidado com todos os controladores.
* **[`matlab/Test.m`](matlab/Test.m):** Análise matemática passo a passo (mapa de polos e zeros, Lugar das Raízes, Diagrama de Bode, Nyquist e Carta de Nichols).
* **[`matlab/teste2.m`](matlab/teste2.m):** Estudo de compensação Dahlin e Dead-Beat.

---

## 📂 Estrutura do Repositório

```text
Sallen-Key/
├── firmware/                              # Código embarcado C/C++
│   └── Sallen-Key-DuinoV2_PID/            # Pasta padrão compatível com Arduino IDE
│       └── Sallen-Key-DuinoV2_PID.ino     # Firmware com chaveamento dinâmico
├── matlab/                                # Scripts MATLAB de simulação e controle
│   ├── Test.m                             # Análise em frequência e estabilidade
│   ├── teste2.m                           # Projeto com Dahlin / Dead-Beat
│   ├── teste3.m                           # Comparativo Teórico vs Medido no Arduino
│   ├── dados/                             # Séries temporais experimentais (.csv)
│   │   ├── Vo.csv / Vo_2.csv              # Ensaio sem controle (malha aberta)
│   │   ├── P_1.csv                        # Ensaio com Proporcional
│   │   ├── AA.csv                         # Ensaio com Avanço de Fase
│   │   ├── PI.csv                         # Ensaio com Proporcional-Integral
│   │   ├── API.csv                        # Ensaio com Avanço + PI
│   │   └── PID.csv                        # Ensaio com PID Robusto
│   └── rascunhos/                         # Rascunhos e versões preliminares (.m)
├── telemetria_python/                     # Scripts de processamento e telemetria
│   ├── grafico.ipynb                      # Notebook Jupyter para plotagem e tratamento
│   ├── *.json                             # Logs brutos do Teleplot
│   └── *.csv                              # Dados estruturados
├── calculos_analiticos/                   # Memórias de cálculo algébrico
│   ├── Página1.sm                         # Planilha do SMath Studio
│   └── Teste_2.xmcd                       # Memória de cálculo no Mathcad 14
├── apresentacao_e_relatorio/              # Documentação acadêmica final
│   ├── Apresentacao_Controladores.pdf     # Slides oficiais da apresentação (22 páginas)
│   └── Controladores Digitais.pdf         # Relatório técnico completo (17 páginas)
├── imagens/                               # Acervo com 29 figuras e gráficos dos ensaios
├── .gitignore                             # Filtros para temporários (MATLAB, Python, etc.)
└── README.md                              # Documentação principal do projeto
```

---

## 🚀 Como Reproduzir o Projeto

### 1. Gravação do Firmware no Arduino:
1. Abra a [Arduino IDE](https://www.arduino.cc/en/software);
2. Conecte o Arduino Nano via USB e selecione a porta COM correspondente;
3. Abra o arquivo [`firmware/Sallen-Key-DuinoV2_PID/Sallen-Key-DuinoV2_PID.ino`](firmware/Sallen-Key-DuinoV2_PID/Sallen-Key-DuinoV2_PID.ino);
4. Faça o upload para a placa.

### 2. Coleta de Telemetria via Teleplot:
1. No VS Code, instale a extensão **Teleplot**;
2. Conecte à porta Serial do Arduino em **115.200 bps**;
3. Envie o comando serial correspondente ao modo desejado (ex: `PID`, `API` ou `SC`);
4. Pressione o botão da placa para aplicar o degrau;
5. Exporte a telemetria gravada para a pasta [`telemetria_python/`](telemetria_python/).

### 3. Validação e Comparação no MATLAB:
1. Abra o MATLAB;
2. Navegue até a pasta [`matlab/`](matlab/);
3. Execute o script:
   ```matlab
   teste3
   ```
4. As figuras comparativas entre os modelos teóricos planejados e os dados práticos do Arduino serão geradas automaticamente.

---

## 📚 Referências Bibliográficas

1. **AGUIRRE, Luis Antonio.** *Controle de Sistemas Amostrados.* Belo Horizonte: Editora UFMG, 2020.
2. **OGATA, Katsuhiko.** *Discrete-Time Control Systems.* 2. ed. Prentice Hall, 1995.
3. **BEZERRA, Daniel.** *Sallen-Key-Duino: Projeto de Hardware e Firmware Didático para Controle.* OSHWLab, 2021. Disponível em: <https://oshwlab.com/daniel_bezerra/sallen-key-duino>.
4. **SOUSA, Luan Italo Mota.** *Repositório Sallen-Key: Modelagem e Controle Digital.* GitHub, 2026. Disponível em: <https://github.com/luanjjbr/Sallen-Key>.
