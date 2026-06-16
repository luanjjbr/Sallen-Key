clc;
clear;
close all;

%% #############################################################
%%   PARÂMETROS GERAIS
%% #############################################################
k   = 0.652;
T   = 10e-3;
ksi = 0.0839;
N   = 61;
ref     = 0.5;
escala  = 500;
Tstep   = 10;

%% #############################################################
%%   1. LEITURA DOS DADOS E MONTAGEM DA PLANTA
%% #############################################################
dados = readtable('Vo.csv');
tempo = dados.tempo;
Vo    = dados.Vo;
tempo = tempo - tempo(1);

wn  = 2*pi/(N*T);
num = k * wn^2;
den = [1, 2*ksi*wn, wn^2];

sysCont = tf(num, den);
G       = c2d(sysCont, T, 'zoh');
z       = tf('z', T);

% --- Sinal bruto ---
figure('Name','Dados experimentais');
plot(tempo, Vo); grid on;
xlabel('Tempo (s)'); ylabel('Vo (0-1000)');
title('Sinal Vo Original Extraído do Arduino');

% --- Comparação modelo x real ---
t_sim  = (0:length(Vo)-1)' * T;
u      = ones(length(Vo),1) * escala;
yModel = lsim(G, u, t_sim);

figure('Name','Modelo x Real');
plot(t_sim, Vo,     'r', 'LineWidth', 1.5); hold on;
plot(t_sim, yModel, 'b', 'LineWidth', 1.5); grid on;
xlabel('Tempo (s)'); ylabel('Amplitude (0-1000)');
legend('Vo Medido','Modelo Discreto','Location','best');
title('Comparação: Vo Real vs Modelo Discreto');

%% =============================================================
%%   SISTEMA 1 - SEM CONTROLE
%% =============================================================
disp(' '); disp('### SISTEMA 1 - SEM CONTROLE ###');
disp('G(s) ='); sysCont
disp('G(z) ='); G

%analisaSistema(G, G, 'SEM CONTROLE', escala, Tstep, ref);

%% =============================================================
%%   SISTEMA 2 - Proporcional (P)
%% =============================================================

%Kp = 1.53;

%disp(' '); disp('### SISTEMA 2 - PROPORCIONAL (P) ###');
%disp('G(s) com P ='); sysCont * Kp
%disp('G(z) com P ='); G * Kp

%MA_P  = G * Kp;

% analisaSistema(MA_P, MA_P, 'PROPORCIONAL (P)', escala, Tstep, ref);

%% =============================================================
%%   SISTEMA 3 - Atraso de Fase
%%   Aguirre eq.(6.16): D(z) = K * (z - z0) / (z - zp)
%%   ATRASO: z0 > zp  (ambos próximos de 1)
%% =============================================================
% Mp = 16
% ξd​=0.6×(1−Mp/100)=0.6×(1−15/100)=0.51

% ωnd​=tr​1.8​=41.8​=0.45 rad/s

z0_at = 0.89;
zp_at = -0.1;
K_at  = 66;

D_base   = tf([1, -z0_at], [1, -zp_at], T);
D_atraso = K_at * D_base;
MA_at    = D_atraso * G;
MF_at    = feedback(MA_at, 1);
analisaSistema(MA_at, MF_at, 'ATRASO DE FASE', escala, Tstep, ref);




%% #############################################################
%%   FUNÇÕES LOCAIS
%% #############################################################
function analisaSistema(L, Tcl, nome, escala, Tstep, ref)

    % --- Step e Polos/Zeros ---
    figure('Name',[nome ' - Resposta e P/Z']);
    subplot(1,2,1);
    step(Tcl*escala, Tstep); grid on;
    title([nome ' - Resposta ao Degrau (x' num2str(escala) ')']);
    xlabel('Tempo (s)'); ylabel('Saída');

    subplot(1,2,2);
    pzmap(Tcl); grid on;
    title([nome ' - Polos e Zeros (MF)']);

    % --- Bode, Nyquist, Nichols ---
    figure('Name',[nome ' - Frequência']);
    subplot(1,3,1); bode(L);    grid on; title([nome ' - Bode']);
    subplot(1,3,2); nyquist(L); grid on; title([nome ' - Nyquist']);
    subplot(1,3,3); nichols(L); grid on; title([nome ' - Nichols']);

    % --- Lugar das Raízes e Margens ---
    figure('Name',[nome ' - LR e Margens']);
    subplot(1,2,1); rlocus(L); grid on; title([nome ' - Lugar das Raízes']);
    subplot(1,2,2); margin(L); grid on; title([nome ' - Margens']);

    % --- Dados numéricos ---
    [Gm, Pm] = margin(L);
    info = stepinfo(Tcl * ref);
    [y, ~] = step(Tcl * ref, Tstep);

    fprintf('\n--- %s ---\n', nome);
    if isinf(Gm)
        fprintf('Margem de Ganho  = Infinita\n');
    else
        fprintf('Margem de Ganho  = %.2f dB\n', 20*log10(Gm));
    end
    fprintf('Margem de Fase   = %.2f graus\n', Pm);
    fprintf('Valor final      = %.4f\n', y(end));
    fprintf('Sobre-sinal      = %.2f %%\n', info.Overshoot);
    fprintf('Tempo acomodacao = %.2f s\n', info.SettlingTime);
    fprintf('Ganho DC (MF)    = %.4f\n', dcgain(Tcl));
    fprintf('Polos de MF:\n'); disp(pole(Tcl));
    fprintf('Módulo dos polos:\n'); disp(abs(pole(Tcl)));
end