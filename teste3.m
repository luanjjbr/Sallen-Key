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
ref     = 500;
ref_2   = 500;
escala  = 1;
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
G = c2d(sysCont, T, 'zoh');
z = tf('z', T);

% --- Sinal bruto ---
figure('Name','Dados experimentais');
plot(tempo, Vo); grid on;
xlabel('Tempo (s)'); ylabel('Vo (0-1000)');
title('Sinal Vo Original Extraído do Arduino');

% --- Comparação modelo x real ---
t_sim  = (0:length(Vo)-1)' * T;
u = ones(length(Vo),1) * ref;
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
disp(' '); disp('sysCont'); disp('--- zpk ---'); zpk(sysCont)
disp(' '); disp('G');       disp('--- zpk ---'); zpk(G)

MA_sc = G;
MF_sc = feedback(G, 1);

figure; rlocus(G); zgrid; title('Lugar das Raízes - Sem Controle')
figure; nyquist(MA_sc); grid on; title('Diagrama de Nyquist - G(z)')
figure; nichols(MA_sc); ngrid; grid on; title('Carta de Nichols - G(z)')
figure; bode(G); grid on; title('Diagrama de Bode - G(z)')
[Gm, Pm, Wcg, Wcp] = margin(G)


%% =============================================================
%%   SISTEMA 7 - COMPARAÇÃO DE TODOS OS CONTROLADORES
%% =============================================================
if true
disp(' ');
disp('############################################################');
disp('###   COMPARAÇÃO GERAL - TODOS OS CONTROLADORES          ###');
disp('############################################################');

% --- (1) PROPORCIONAL ---
Kp_P = 1.53;
C_P  = Kp_P;
MA_P = C_P*G;
MF_P = feedback(C_P*G, 1);

% --- (2) AVANÇO ---
zc_av = 0.85;  pc_av = -0.85;  Kc_av = 150;
C_AV  = tf(Kc_av*[1 -zc_av], [1 -pc_av], T);
MF_AV = feedback(C_AV*G, 1);

% --- (3) PI ROBUSTO ---  OS=0.9% ts~5.3s MG=6dB MF=96°
zc_pi = 0.925;  Kc_pi = 0.20;
C_PI  = Kc_pi * tf([1 -zc_pi],[1 -1], T);
MF_PI = feedback(C_PI*G, 1);

% --- (4) AVANÇO + PI ROBUSTO ---  OS=0% ts~3s MG=34dB MF=39°
zc_av2 = 0.90;  pc_av2 = 0.30;  Kc_av2 = 2;
C_lead = tf(Kc_av2*[1 -zc_av2], [1 -pc_av2], T);
zc_pi2 = 0.95;  Kc_pi2 = 1.90;
C_pi2  = Kc_pi2 * tf([1 -zc_pi2],[1 -1], T);
MF_AVPI = feedback(C_pi2*C_lead*G, 1);

% --- (5) PID ROBUSTO ---  OS=0% ts~2.95s MG=40dB MF=97° (MELHOR)
Kp_pid = 0.25;  Ki_pid = 2.25;  Kd_pid = 0.025;
Cp = tf(Kp_pid, 1, T);
Ci = tf(Ki_pid*T, [1 -1], T);
Cd = tf(Kd_pid/T*[1 -1], [1 0], T);
C_PID  = Cp + Ci + Cd;
MF_PID = feedback(C_PID*G, 1);

% =============================================================
%   ESCOLHA QUAIS SISTEMAS APARECEM  (true = mostra | false = oculta)
% =============================================================
mostra_SC   = true;    % Sem Controle
mostra_P    = true;    % Proporcional
mostra_AV   = true;    % Avanço
mostra_PI   = true;    % PI
mostra_AVPI = true;    % Avanço+PI
mostra_PID  = true;    % PID

% --- Monta as listas só com os selecionados ---
todos_MF   = {MA_sc, MF_P, MF_AV, MF_PI, MF_AVPI, MF_PID};
todos_nome = {'Sem Controle','Proporcional','Avanço','PI','Avanço+PI','PID'};
todos_cor  = {'k','b','g','c',[0.8 0.5 0],'r'};
todos_MA   = {G, C_P*G, C_AV*G, C_PI*G, C_pi2*C_lead*G, C_PID*G};
sel = [mostra_SC, mostra_P, mostra_AV, mostra_PI, mostra_AVPI, mostra_PID];

sistemas = {};  nomes = {};  cores = {};  MA_list = {};
for i = 1:6
    if sel(i)
        sistemas{end+1} = todos_MF{i};
        nomes{end+1}    = todos_nome{i};
        cores{end+1}    = todos_cor{i};
        MA_list{end+1}  = todos_MA{i};
    end
end

% =============================================================
%   GRÁFICO 1 - RESPOSTA AO DEGRAU
% =============================================================
figure('Name','Comparação - Resposta ao Degrau'); hold on
for i = 1:length(sistemas)
    [y, t] = step(ref*sistemas{i});
    plot(t, y, 'Color', cores{i}, 'LineWidth', 1.4)
end
plot(tempo, Vo, 'm--', 'LineWidth', 1.0)
yline(ref,'--k')
grid on
legend([nomes, {'Vo Medido','Referência'}], 'Location','best')
xlabel('Tempo (s)'); ylabel('Amplitude (0-1000)')
title('Comparação - Resposta ao Degrau')
xlim([0 6])
hold off

% =============================================================
%   GRÁFICO 2 - LUGAR DAS RAÍZES
% =============================================================
figure('Name','Comparação - Lugar das Raízes'); hold on
for i = 1:length(MA_list)
    rlocus(MA_list{i})
    h = findobj(gca,'Type','line'); set(h(1),'Color',cores{i});
end
zgrid
xlim([-1.5 1.5])        % limita eixo real
ylim([-1.5 1.5])        % limita eixo imaginário
axis equal              % mantém o círculo unitário redondo
legend(nomes, 'Location','best')
title('Comparação - Lugar das Raízes')
hold off

% =============================================================
%   GRÁFICO 3 - BODE
% =============================================================
figure('Name','Comparação - Bode'); hold on
for i = 1:length(MA_list)
    bode(MA_list{i})
    h = findobj(gca,'Type','line'); set(h(1),'Color',cores{i});
end
grid on
legend(nomes, 'Location','best')
title('Comparação - Bode')
hold off

% =============================================================
%   GRÁFICO 4 - NYQUIST
% =============================================================
figure('Name','Comparação - Nyquist'); hold on
for i = 1:length(MA_list)
    nyquist(MA_list{i})
    h = findobj(gca,'Type','line'); set(h(1),'Color',cores{i});
end
grid on
legend(nomes, 'Location','best')
title('Comparação - Nyquist')
hold off

% =============================================================
%   GRÁFICO 5 - NICHOLS
% =============================================================
figure('Name','Comparação - Nichols'); hold on
for i = 1:length(MA_list)
    nichols(MA_list{i})
    h = findobj(gca,'Type','line'); set(h(1),'Color',cores{i});
end
ngrid
legend(nomes, 'Location','best')
title('Comparação - Nichols')
hold off

% =============================================================
%   GRÁFICO 6 - PZMAP (polos e zeros de malha fechada)
% =============================================================
figure('Name','Comparação - PZmap'); hold on
for i = 1:length(sistemas)
    pzmap(sistemas{i});
    hp = findobj(gca, 'Type', 'line');
    set(hp(1:min(2,end)), 'Color', cores{i});
end
zgrid
legend(nomes, 'Location','best')
title('Comparação - Polos e Zeros de Malha Fechada')
hold off

% =============================================================
%   TABELA DE DESEMPENHO (com margens de estabilidade)
% =============================================================
disp(' ');
disp('### TABELA COMPARATIVA DE DESEMPENHO ###');
fprintf('%-14s %9s %10s %8s %8s %8s %8s %8s\n', ...
        'Controlador','RiseT(s)','SettleT(s)','OS(%)','GanhoDC','Erro(%)','MG(dB)','MF(deg)');
fprintf('%s\n', repmat('-',1,80));
for i = 1:length(sistemas)
    info = stepinfo(sistemas{i});
    gdc  = dcgain(sistemas{i});
    erro = (1 - gdc)*100;          % erro de regime permanente (%)
    [Gm, Pm] = margin(MA_list{i});
    Gm_db = 20*log10(Gm);
    fprintf('%-14s %9.4g %10.4g %8.4g %8.4g %8.4g %8.4g %8.4g\n', ...
            nomes{i}, info.RiseTime, info.SettlingTime, ...
            info.Overshoot, gdc, erro, Gm_db, Pm);
end
fprintf('%s\n', repmat('-',1,80));
disp('Obs: GanhoDC = 1 (Erro = 0) -> rastreamento perfeito');
disp('     Erro(%) = (1 - GanhoDC)*100 -> erro de regime ao degrau');
disp('     MG = Inf -> sistema de 2a ordem sem atraso (margem de ganho infinita)');
end













if false
%% =============================================================
%%   SISTEMA 8 - DEAD-BEAT (suavizado / Dahlin) - VIÁVEL ts=6s
%% =============================================================
disp(' '); disp('### SISTEMA 8 - DEAD-BEAT (ts=6s, não satura PWM) ###');

% --- Projeto: malha fechada = 1a ordem com ts desejado ---
% ts=1s satura o atuador (esforço ~5.7). ts=6s -> esforço ~0.97 (viável)
ts_des = 6.0;                  % tempo de acomodação desejado (s)
tau    = ts_des/4;             % constante de tempo (ts ~ 4*tau)
q      = exp(-T/tau);          % polo desejado de malha fechada

% Resposta de malha fechada desejada: T_d(z) = (1-q)/(z-q)
Td = tf([1-q], [1 -q], T);

% Controlador: D(z) = T_d / [G*(1 - T_d)]
C_DB = minreal(Td / (G*(1 - Td)));

MA_DB = C_DB*G;
MF_DB = feedback(C_DB*G, 1);   % CORRIGIDO: feedback(C_DB*G, 1) com a planta G

disp('Controlador Dead-Beat (Dahlin) D(z):'); zpk(C_DB)
fprintf('Polo de malha fechada desejado q = %.4f\n', q);
disp('Ganho DC malha fechada:'); disp(dcgain(MF_DB))

% --- Coeficientes para o Arduino ---
[B_DB, A_DB] = tfdata(C_DB, 'v');
disp('Coeficientes p/ Arduino (u = -a1*u1 -a2*u2 + b0*e + b1*e1 + b2*e2):');
fprintf('  DB_b0 = %.5f\n', B_DB(1));
fprintf('  DB_b1 = %.5f\n', B_DB(2));
fprintf('  DB_b2 = %.5f\n', B_DB(3));
fprintf('  DB_a1 = %.5f\n', A_DB(2));
fprintf('  DB_a2 = %.5f\n', A_DB(3));

% --- Esforço de controle (verifica se satura) ---
Du = minreal(C_DB/(1+C_DB*G));
u_db = lsim(Du, ref/1000*ones(size(0:T:2)), 0:T:2);
fprintf('Pico do esforço de controle: %.2f (satura se > 1.0)\n', max(abs(u_db)));

% --- Resposta ao degrau ---
figure;
step(MA_sc*ref,'b'); hold on
step(MF_DB*ref,'r')
plot(tempo, Vo, 'm')
yline(ref,'--k'); grid on
legend('Sem Controle','Dead-Beat','Vo Medido','Referência')
title('Resposta ao Degrau - Dead-Beat (ts=6s)')
xlim([0 8])
hold off

% --- Desempenho ---
info_DB = stepinfo(MF_DB);
disp('### DESEMPENHO DEAD-BEAT ###');
fprintf('Tempo de subida    : %.4g s\n', info_DB.RiseTime);
fprintf('Tempo de acomodacao: %.4g s\n', info_DB.SettlingTime);
fprintf('Sobressinal        : %.4g %%\n', info_DB.Overshoot);

% --- Polos/zeros e frequência ---
figure; pzmap(MF_DB); grid on; title('Polos e Zeros - Dead-Beat')
figure; margin(MA_DB); grid on; title('Bode - Dead-Beat (com margens)')
figure; nyquist(MA_sc,'b',MA_DB,'r'); grid on
legend('Sem Controle','Dead-Beat'); title('Nyquist - Comparação')
figure; nichols(MA_sc,'b',MA_DB,'r'); ngrid
legend('Sem Controle','Dead-Beat'); title('Nichols - Comparação')
end

if false
dados = readtable('PID.csv');
tempo = dados.tempo;
Vo    = dados.Vo;

% Acha o primeiro valor diferente de zero e corta tudo antes
idx = find(Vo ~= 0, 1);
tempo = tempo(idx:end);
Vo    = Vo(idx:end);
tempo = tempo - tempo(1);

figure;
[y_AA, t_AA] = step(ref*MF_DB);     % captura os dados (evita erro de tipo)
plot(t_AA, y_AA, 'c', 'LineWidth', 1.5); hold on
plot(tempo, Vo, 'm', 'LineWidth', 1.2)
yline(ref,'--k'); grid on
legend('(planejado)','Vo Medido (Arduino)','Referência','Location','best')
title('Avanço: Planejado vs Arduino')
xlim([0 max(tempo)])
hold off
end














if false
%% =============================================================
%%   COMPARAÇÃO PLANEJADO vs ARDUINO
%% =============================================================
nomes_c  = {'SC','P','AA','PI','API','PID'};
MF_c     = {MA_sc, MF_P, MF_AV, MF_PI, MF_AVPI, MF_PID};
arquivos = {'Vo_2.csv','P_1.csv','AA.csv','PI.csv','API.csv','PID.csv'};
cores_c  = {'k','b','g','c',[0.8 0.5 0],'r'};

% =============================================================
%   GRÁFICOS INDIVIDUAIS (um por controlador)
% =============================================================
for j = 1:length(nomes_c)
    if ~isfile(arquivos{j})
        warning('Arquivo %s não encontrado, pulando.', arquivos{j});
        continue
    end
    dados = readtable(arquivos{j});
    tp = dados.tempo;  vo = dados.Vo;
    idx = find(vo ~= 0, 1);
    tp = tp(idx:end) - tp(idx);
    vo = vo(idx:end);

    figure('Name',['Planejado vs Arduino - ' nomes_c{j}]);
    [y_sim, t_sim2] = step(ref*MF_c{j});
    plot(t_sim2, y_sim, 'c', 'LineWidth', 1.5); hold on
    plot(tp, vo, 'm', 'LineWidth', 1.2)
    yline(ref,'--k'); grid on
    legend([nomes_c{j} ' (planejado)'],'Vo Medido (Arduino)','Referência','Location','best')
    title([nomes_c{j} ': Planejado vs Arduino'])
    xlim([0 max(tp)])
    hold off
end

% =============================================================
%   TODOS EM UM GRÁFICO (Arduino = linha cheia | Planejado = ---)
% =============================================================
figure('Name','Planejado vs Arduino - Todos'); hold on
legendas = {};
tmax = 0;
for j = 1:length(nomes_c)
    cor = cores_c{j};

    % --- Planejado (tracejada ---) ---
    [y_sim, t_sim2] = step(ref*MF_c{j});
    plot(t_sim2, y_sim, '--', 'Color', cor, 'LineWidth', 1.3)
    legendas{end+1} = [nomes_c{j} ' (planejado)'];

    % --- Arduino (linha cheia) ---
    if isfile(arquivos{j})
        dados = readtable(arquivos{j});
        tp = dados.tempo;  vo = dados.Vo;
        idx = find(vo ~= 0, 1);
        tp = tp(idx:end) - tp(idx);
        vo = vo(idx:end);
        plot(tp, vo, '-', 'Color', cor, 'LineWidth', 1.6)
        legendas{end+1} = [nomes_c{j} ' (Arduino)'];
        tmax = max(tmax, max(tp));
    end
end
yline(ref,'--k'); legendas{end+1} = 'Referência';
grid on
xlabel('Tempo (s)'); ylabel('Amplitude (0-1000)')
legend(legendas, 'Location','best')
title('Comparação - Planejado vs Arduino (todos)')
if tmax > 0, xlim([0 tmax]); end
hold off
end