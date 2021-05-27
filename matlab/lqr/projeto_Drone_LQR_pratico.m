

% PROJETO DOS CONTROLADORES LQR
% com aloca��o de polos (dados identificados)

close all
clc

K_fi_min = 1;
K_fi_max = 2;

K_fi = (K_fi_max+K_fi_min)/2;

Fi_max = 0.4171/2;

% K_fi = 1;
% Fi_max = 23.9*(pi/180);
% Fi_min = (23.9/2)*(pi/180);

omega_fi = 14.43;

d_fi = 1.35;

K_teta_min = 1;
K_teta_max = 2;

K_teta = (K_teta_max+K_teta_min)/2;

Teta_max = 0.4184/2;

% K_teta = 1;
% Teta_max = 23.97*(pi/180);
% Teta_min = (23.9/2)*(pi/180);

omega_teta = 13.14;

d_teta = 0.99;

K_psi_p = 1;
Psi_p_max = 92.75*(pi/180);

Tau_psi_p = 0.51;

K_z_p = 1;
z_p_max = 1.35 ;


Tau_z_p = 0.88;%

g =0.165/(pi/180);%

Cx = 1.33;

Cy = 1.95;

%======================

K_teta_min = 1;
K_teta_max = 2;

K_teta = (K_teta_max+K_teta_min)/2;

Teta_max = 0.4184/2;

T = 5;


% Modelo no espa�o de estado para fi/y

A_fi = [0 1 0 0;
       -omega_fi^2 -2*d_fi*omega_fi 0 0;
       0 0 0 1;
       -g 0 0 -Cy];

B2_fi = [0 K_fi*Fi_max*(omega_fi^2) 0 0]';

C_fi = [0 0 1 0];

D_fi = 0;

sis_fi = ss(A_fi,B2_fi,C_fi,D_fi);



% Modelo no espa�o de estado para teta/x

A_teta = [0 1 0 0;
       -omega_teta^2 -2*d_teta*omega_teta 0 0;
       0 0 0 1;
       g 0 0 -Cx];

B2_teta = [0 K_teta*Teta_max*(omega_teta^2) 0 0]';

C_teta = [0 0 1 0];

D_teta = 0;

sis_teta = ss(A_teta,B2_teta,C_teta,D_teta);



% Modelo no espa�o de estado para psi

A_psi = [0 1;
         0 -1/Tau_psi_p];

B2_psi = [0 K_psi_p*Psi_p_max/Tau_psi_p]';

C_psi = [1 0];

D_psi = 0;

sis_psi = ss(A_psi,B2_psi,C_psi,D_psi);



% Modelo no espa�o de estado para z

A_z = [0 1;
         0 -1/Tau_z_p];

B2_z = [0 K_z_p*z_p_max/Tau_z_p]';

C_z = [1 0];

D_z = 0;

sis_z = ss(A_z,B2_z,C_z,D_z);


%===========================================
%===========================================
% Projeto do controlador LQR para a malha
% fi/y

ro_fi = 0.1;%0.0001;
%ro_fi = 0.000001;

Q_fi = [eye(4) zeros(4,1);
        zeros(1,4) 100];

R_fi = ro_fi;

N_fi = 0;

[Ka_y,S,e] = lqi(sis_fi,Q_fi,R_fi,N_fi);

Ky = -Ka_y(1,1:4)
Kiy = -Ka_y(1,5)

% Projeto do controlador LQR para a malha
% teta/x

ro_teta = 0.1; %0.0001;
%ro_teta = 0.000001;

Q_teta = [eye(4) zeros(4,1);
        zeros(1,4) 100];

R_teta = ro_teta;

N_teta = 0;

[Ka_x,S,e] = lqi(sis_teta,Q_teta,R_teta,N_teta);

Kx = -Ka_x(1,1:4)
Kix = -Ka_x(1,5)


% Projeto do controlador LQR para a malha
% psi

ro_psi = 0.1;

Q_psi = [1 0 0;
         0 1 0;
         0 0 10];

R_psi = ro_psi;

N_psi = 0;

[Ka_psi,S,e] = lqi(sis_psi,Q_psi,R_psi,N_psi);

Kpsi = -Ka_psi(1,1:2)
Kipsi = -Ka_psi(1,3)

% Projeto do controlador LQR para a malha
% z

ro_z = 0.1;
%ro_z = 0.000001;

Q_z = [1 0 0;
       0 1 0;
       0 0 10];

R_z = ro_z;

N_z = 0;

[Ka_z,S,e] = lqi(sis_z,Q_z,R_z,N_z);

Kz = -Ka_z(1,1:2)
Kiz = -Ka_z(1,3)

%=================================

sim('drone_nao_linear_V3_MF_LQR')
%sim('drone_nao_linear_V3_MF_LQR8')
graficos