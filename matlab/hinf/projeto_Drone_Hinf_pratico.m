
% PROJETO DOS CONTROLADORES ROBUSTOS Hinfinito
% com alocação de polos (dados identificados)

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


% Modelo no espaço de estado para fi/y

A_fi1 = [0 1 0 0;
       -omega_fi^2 -2*d_fi*omega_fi 0 0;
       0 0 0 1;
       -g 0 0 -Cy];

B2_fi1 = [0 K_fi*Fi_max*(omega_fi^2) 0 0]';

C2_fi1 = [0 0 1 0];

D22_fi1 = 0;

D21_fi1 = 0;

B1_fi1 = B2_fi1;

% Sistema aumentado

A_fi = [];
A_fi(:,:,1) = [A_fi1 zeros(4,1); 
              -C2_fi1 zeros(1,1)];

%A_fi(:,:,2) = ;

B2_fi = [];
B2_fi(:,:,1) = [B2_fi1; 
                0];
%B2_fi(:,:,2) = ;

B1_fi = [];
B1_fi(:,:,1) = [B1_fi1; 
                0];
%B1_f1(:,:,2) = ;

C2_fi = [C2_fi1 0];

D22_fi = D22_fi1;
D21_fi = D21_fi1;

C1_fi = C2_fi;
D12_fi = D22_fi;
D11_fi = D21_fi;


% Modelo no espaço de estado para teta/x

A_teta1 = [0 1 0 0;
       -omega_teta^2 -2*d_teta*omega_teta 0 0;
       0 0 0 1;
       g 0 0 -Cx];

B2_teta1 = [0 K_teta*Teta_max*(omega_teta^2) 0 0]';

C2_teta1 = [0 0 1 0];

D22_teta1 = 0;

D21_teta1 = 0;

B1_teta1 = B2_teta1;

% Sistema aumentado

A_teta = [];
A_teta(:,:,1) = [A_teta1 zeros(4,1); 
                 -C2_teta1 zeros(1,1)];

%A_teta(:,:,2) = ;

B2_teta = [];
B2_teta(:,:,1) = [B2_teta1; 
                  0];
%B2_teta(:,:,2) = ;

B1_teta = [];
B1_teta(:,:,1) = [B1_teta1; 
                0];
%B1_teta(:,:,2) = ;

C2_teta = [C2_teta1 0];

D22_teta = D22_teta1;
D21_teta = D21_teta1;

C1_teta = C2_teta;
D12_teta = D22_teta;
D11_teta = D21_teta;




% Modelo no espaço de estado para psi

A_psi1 = [0 1;
         0 -1/Tau_psi_p];

B2_psi1 = [0 K_psi_p*Psi_p_max/Tau_psi_p]';

C2_psi1 = [1 0];

D22_psi1 = 0;
D21_psi1 = 0;

B1_psi1 = B2_psi1;


% Sistema aumentado

A_psi = [];
A_psi(:,:,1) = [A_psi1 zeros(2,1); 
                 -C2_psi1 zeros(1,1)];

%A_psi(:,:,2) = ;

B2_psi = [];
B2_psi(:,:,1) = [B2_psi1; 
                  0];
%B2_psi(:,:,2) = ;

B1_psi = [];
B1_psi(:,:,1) = [B1_psi1; 
                0];
%B1_psi(:,:,2) = ;

C2_psi = [C2_psi1 0];

D22_psi = D22_psi1;
D21_psi = D21_psi1;


C1_psi = C2_psi;
D12_psi = D22_psi;
D11_psi = D21_psi;





% Modelo no espaço de estado para z

A_z1 = [0 1;
         0 -1/Tau_z_p];

B2_z1 = [0 K_z_p*z_p_max/Tau_z_p]';

C2_z1 = [1 0];

D22_z1 = 0;
D21_z1 = 0;

B1_z1 = B2_z1;

% Sistema aumentado

A_z = [];
A_z(:,:,1) = [A_z1 zeros(2,1); 
              -C2_z1 zeros(1,1)];

%A_z(:,:,2) = ;

B2_z = [];
B2_z(:,:,1) = [B2_z1; 
                0];
%B2_z(:,:,2) = ;

B1_z = [];
B1_z(:,:,1) = [B1_z1; 
                0];
%B1_z(:,:,2) = ;

C2_z = [C2_z1 0];

D22_z = D22_z1;
D21_z = D21_z1;

C1_z = C2_z;
D12_z = D22_z;
D11_z = D21_z;




%==================================================
%==================================================
% Projeto do controlador robusto Hinf para a malha
% fi/y

alfa_fi = 2;
bet_fi = 8;
teta_fi = 25*pi/180;
aloc_fi = 1;


%==========================================================================
%==========================================================================

%[ny,nn] = size(D);       % ny : output vector dimension
[nn,m,nb] = size(B2_fi);       % nu : control vector dimension
[n,nn,na] = size(A_fi);         % n : state vector dimension


%==========================================================================
%======================================================================
%==========================================================================
% PROBLEMA Hinf e LMIs


setlmis([]);

% Declaração das variáveis

gama_fi = lmivar(1,[1 1]);

[X_fi,n_Xfi] = lmivar(1,[n 1]);
[L_fi,n_Lfi] = lmivar(2,[m n]);

%======================================================================
% Restrições LMIs: norma Hinf
%

j = 1;

for i=1:na
    for k=1:nb
        lmiterm([j 1 1 X_fi],A_fi(:,:,i),1,'s');
        lmiterm([j 1 1 L_fi],B2_fi(:,:,k),1,'s');
        lmiterm([j 2 1 0],B1_fi');
        lmiterm([j 2 2 0],-1);
        lmiterm([j 3 1 X_fi],C1_fi,1);
        lmiterm([j 3 1 L_fi],D12_fi,1);
        lmiterm([j 3 2 0],D11_fi);
        lmiterm([j 3 3 gama_fi],-1,1)
        j=j+1;
    end
end

%======================================================================
% Restrições de alocação

if aloc_fi==1
    
    % LMIs de alocação em uma faixa vertical
    j = j+1;
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_fi],A_fi(:,:,i),1,'s');
            lmiterm([j 1 1 L_fi],B2_fi(:,:,k),1,'s');
            lmiterm([j 1 1 X_fi],2*alfa_fi,1);
            j=j+1;
        end
    end


    j = j+1;
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_fi],-A_fi(:,:,i),1,'s');
            lmiterm([j 1 1 L_fi],-B2_fi(:,:,k),1,'s');
            lmiterm([j 1 1 X_fi],-2*bet_fi,1);
            j=j+1;
        end
    end

    
    j = j+1;
    
    %LMI de alocação no setor cônico
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_fi],sin(teta_fi)*A_fi(:,:,i),1,'s');
            lmiterm([j 1 1 L_fi],sin(teta_fi)*B2_fi(:,:,k),1,'s');
            lmiterm([j 2 1 X_fi],-cos(teta_fi)*A_fi(:,:,i),1);
            lmiterm([j 2 1 X_fi],1,cos(teta_fi)*A_fi(:,:,i)');
            lmiterm([j 2 1 L_fi],-cos(teta_fi)*B2_fi(:,:,k),1);
            lmiterm([j 2 1 -L_fi],1,cos(teta_fi)*B2_fi(:,:,k)');
            lmiterm([j 2 2 X_fi],sin(teta_fi)*A_fi(:,:,i),1,'s');
            lmiterm([j 2 2 L_fi],sin(teta_fi)*B2_fi(:,:,k),1,'s');
            j=j+1;
        end
    end
    
    % LMI de alocação no círculo
    
    %     j = j+1;
    %     lmiterm([j 1 1 W1],A(:,:,1),1,'s');
    %     lmiterm([j 1 1 W2],B2(:,:,1),1,'s');
    %     lmiterm([j 1 1 W1],2*alfa,1);
    %     lmiterm([j 1 2 W1],A(:,:,1),1);
    %     lmiterm([j 1 2 W2],B2(:,:,1),1);
    %     lmiterm([j 1 2 W1],alfa,1);
    %     lmiterm([j 2 2 W1],-r,1);
    
end
%======================================================================
% GAMA > 0

j = j+1;
lmiterm([-j 1 1 gama_fi],1,1);

%======================================================================

j = j+1;

lmiterm([-j 1 1 X_fi],1,1);

%======================================================================

Controlador_Hinf_fi = getlmis;


%======================================================================
%======================================================================
% Análise da norma Hinf

%c = mat2dec(LQR_lmi,0,0,0,1);
c=[1 zeros(1,n_Lfi-1)]';

options = [1e-8,3000,1e9,1e-4,1];
[c_optfi,x_optfi] = mincx(Controlador_Hinf_fi,c, options);


gama_optfi = dec2mat(Controlador_Hinf_fi,x_optfi,gama_fi);
X_optfi   = dec2mat(Controlador_Hinf_fi,x_optfi,X_fi);
L_optfi   = dec2mat(Controlador_Hinf_fi,x_optfi,L_fi);


% Critério

disp('Valor do norma Hinf via LMI:')

sqrt(gama_optfi)


disp('Controlador Hinf via LMI')
Ka_y = L_optfi*inv(X_optfi);

Ky = Ka_y(1,1:4)
Kiy = Ka_y(1,5)




% Projeto do controlador robusto Hinf para a malha
% teta/x

alfa_teta = 2;
bet_teta = 8;
teta_teta = 25*pi/180;
aloc_teta = 1;


%==========================================================================
%==========================================================================

%[ny,nn] = size(D);       % ny : output vector dimension
[nn,m,nb] = size(B2_teta);       % nu : control vector dimension
[n,nn,na] = size(A_teta);         % n : state vector dimension


%==========================================================================
%======================================================================
%==========================================================================
% PROBLEMA Hinf e LMIs


setlmis([]);

% Declaração das variáveis

gama_teta = lmivar(1,[1 1]);

[X_teta,n_Xteta] = lmivar(1,[n 1]);
[L_teta,n_Lteta] = lmivar(2,[m n]);

%======================================================================
% Restrições LMIs: norma Hinf
%

j = 1;

for i=1:na
    for k=1:nb
        lmiterm([j 1 1 X_teta],A_teta(:,:,i),1,'s');
        lmiterm([j 1 1 L_teta],B2_teta(:,:,k),1,'s');
        lmiterm([j 2 1 0],B1_teta');
        lmiterm([j 2 2 0],-1);
        lmiterm([j 3 1 X_teta],C1_teta,1);
        lmiterm([j 3 1 L_teta],D12_teta,1);
        lmiterm([j 3 2 0],D11_teta);
        lmiterm([j 3 3 gama_teta],-1,1)
        j=j+1;
    end
end

%======================================================================
% Restrições de alocação

if aloc_teta==1
    
    % LMIs de alocação em uma faixa vertical
    j = j+1;
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_teta],A_teta(:,:,i),1,'s');
            lmiterm([j 1 1 L_teta],B2_teta(:,:,k),1,'s');
            lmiterm([j 1 1 X_teta],2*alfa_teta,1);
            j=j+1;
        end
    end


    j = j+1;
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_teta],-A_teta(:,:,i),1,'s');
            lmiterm([j 1 1 L_teta],-B2_teta(:,:,k),1,'s');
            lmiterm([j 1 1 X_teta],-2*bet_teta,1);
            j=j+1;
        end
    end

    
    j = j+1;
    
    %LMI de alocação no setor cônico
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_teta],sin(teta_teta)*A_teta(:,:,i),1,'s');
            lmiterm([j 1 1 L_teta],sin(teta_teta)*B2_teta(:,:,k),1,'s');
            lmiterm([j 2 1 X_teta],-cos(teta_teta)*A_teta(:,:,i),1);
            lmiterm([j 2 1 X_teta],1,cos(teta_teta)*A_teta(:,:,i)');
            lmiterm([j 2 1 L_teta],-cos(teta_teta)*B2_teta(:,:,k),1);
            lmiterm([j 2 1 -L_teta],1,cos(teta_teta)*B2_teta(:,:,k)');
            lmiterm([j 2 2 X_teta],sin(teta_teta)*A_teta(:,:,i),1,'s');
            lmiterm([j 2 2 L_teta],sin(teta_teta)*B2_teta(:,:,k),1,'s');
            j=j+1;
        end
    end
    
    % LMI de alocação no círculo
    
    %     j = j+1;
    %     lmiterm([j 1 1 W1],A(:,:,1),1,'s');
    %     lmiterm([j 1 1 W2],B2(:,:,1),1,'s');
    %     lmiterm([j 1 1 W1],2*alfa,1);
    %     lmiterm([j 1 2 W1],A(:,:,1),1);
    %     lmiterm([j 1 2 W2],B2(:,:,1),1);
    %     lmiterm([j 1 2 W1],alfa,1);
    %     lmiterm([j 2 2 W1],-r,1);
    
end
%======================================================================
% GAMA > 0

j = j+1;
lmiterm([-j 1 1 gama_teta],1,1);

%======================================================================

j = j+1;

lmiterm([-j 1 1 X_teta],1,1);

%======================================================================

Controlador_Hinf_teta = getlmis;


%======================================================================
%======================================================================
% Análise da norma Hinf

%c = mat2dec(LQR_lmi,0,0,0,1);
c=[1 zeros(1,n_Lteta-1)]';

options = [1e-8,3000,1e9,1e-4,1];
[c_optteta,x_optteta] = mincx(Controlador_Hinf_teta,c, options);


gama_optteta = dec2mat(Controlador_Hinf_teta,x_optteta,gama_teta);
X_optteta   = dec2mat(Controlador_Hinf_teta,x_optteta,X_teta);
L_optteta   = dec2mat(Controlador_Hinf_teta,x_optteta,L_teta);


% Critério

disp('Valor do norma Hinf via LMI:')

sqrt(gama_optteta)


disp('Controlador Hinf via LMI')
Ka_x = L_optteta*inv(X_optteta);


Kx = Ka_x(1,1:4)
Kix = Ka_x(1,5)


% Projeto do controlador robusto Hinf para a malha
% psi

alfa_psi = 2;
bet_psi = 4;
teta_psi = 20*pi/180;
aloc_psi = 1;

%==========================================================================
%==========================================================================

%[ny,nn] = size(D);       % ny : output vector dimension
[nn,m,nb] = size(B2_psi);       % nu : control vector dimension
[n,nn,na] = size(A_psi);         % n : state vector dimension


%==========================================================================
%======================================================================
%==========================================================================
% PROBLEMA Hinf e LMIs


setlmis([]);

% Declaração das variáveis

gama_psi = lmivar(1,[1 1]);

[X_psi,n_Xpsi] = lmivar(1,[n 1]);
[L_psi,n_Lpsi] = lmivar(2,[m n]);

%======================================================================
% Restrições LMIs: norma Hinf
%

j = 1;

for i=1:na
    for k=1:nb
        lmiterm([j 1 1 X_psi],A_psi(:,:,i),1,'s');
        lmiterm([j 1 1 L_psi],B2_psi(:,:,k),1,'s');
        lmiterm([j 2 1 0],B1_psi');
        lmiterm([j 2 2 0],-1);
        lmiterm([j 3 1 X_psi],C1_psi,1);
        lmiterm([j 3 1 L_psi],D12_psi,1);
        lmiterm([j 3 2 0],D11_psi);
        lmiterm([j 3 3 gama_psi],-1,1)
        j=j+1;
    end
end

%======================================================================
% Restrições de alocação

if aloc_psi==1
    
    % LMIs de alocação em uma faixa vertical
    j = j+1;
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_psi],A_psi(:,:,i),1,'s');
            lmiterm([j 1 1 L_psi],B2_psi(:,:,k),1,'s');
            lmiterm([j 1 1 X_psi],2*alfa_psi,1);
            j=j+1;
        end
    end


    j = j+1;
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_psi],-A_psi(:,:,i),1,'s');
            lmiterm([j 1 1 L_psi],-B2_psi(:,:,k),1,'s');
            lmiterm([j 1 1 X_psi],-2*bet_psi,1);
            j=j+1;
        end
    end

    
    j = j+1;
    
    %LMI de alocação no setor cônico
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_psi],sin(teta_psi)*A_psi(:,:,i),1,'s');
            lmiterm([j 1 1 L_psi],sin(teta_psi)*B2_psi(:,:,k),1,'s');
            lmiterm([j 2 1 X_psi],-cos(teta_psi)*A_psi(:,:,i),1);
            lmiterm([j 2 1 X_psi],1,cos(teta_psi)*A_psi(:,:,i)');
            lmiterm([j 2 1 L_psi],-cos(teta_psi)*B2_psi(:,:,k),1);
            lmiterm([j 2 1 -L_psi],1,cos(teta_psi)*B2_psi(:,:,k)');
            lmiterm([j 2 2 X_psi],sin(teta_psi)*A_psi(:,:,i),1,'s');
            lmiterm([j 2 2 L_psi],sin(teta_psi)*B2_psi(:,:,k),1,'s');
            j=j+1;
        end
    end
    
    % LMI de alocação no círculo
    
    %     j = j+1;
    %     lmiterm([j 1 1 W1],A(:,:,1),1,'s');
    %     lmiterm([j 1 1 W2],B2(:,:,1),1,'s');
    %     lmiterm([j 1 1 W1],2*alfa,1);
    %     lmiterm([j 1 2 W1],A(:,:,1),1);
    %     lmiterm([j 1 2 W2],B2(:,:,1),1);
    %     lmiterm([j 1 2 W1],alfa,1);
    %     lmiterm([j 2 2 W1],-r,1);
    
end
%======================================================================
% GAMA > 0

j = j+1;
lmiterm([-j 1 1 gama_psi],1,1);

%======================================================================

j = j+1;

lmiterm([-j 1 1 X_psi],1,1);

%======================================================================

Controlador_Hinf_psi = getlmis;


%======================================================================
%======================================================================
% Análise da norma Hinf

%c = mat2dec(LQR_lmi,0,0,0,1);
c=[1 zeros(1,n_Lpsi-1)]';

options = [1e-8,3000,1e9,1e-4,1];
[c_optpsi,x_optpsi] = mincx(Controlador_Hinf_psi,c, options);


gama_optpsi = dec2mat(Controlador_Hinf_psi,x_optpsi,gama_psi);
X_optpsi   = dec2mat(Controlador_Hinf_psi,x_optpsi,X_psi);
L_optpsi   = dec2mat(Controlador_Hinf_psi,x_optpsi,L_psi);


% Critério

disp('Valor do norma Hinf via LMI:')

sqrt(gama_optpsi)


disp('Controlador Hinf via LMI')
Ka_psi = L_optpsi*inv(X_optpsi);


Kpsi = Ka_psi(1,1:2)
Kipsi = Ka_psi(1,3)



% Projeto do controlador robusto Hinf para a malha
% z

alfa_z = 2;
bet_z = 4;
teta_z = 10*pi/180;
aloc_z = 1;

%==========================================================================
%==========================================================================

%[ny,nn] = size(D);       % ny : output vector dimension
[nn,m,nb] = size(B2_z);       % nu : control vector dimension
[n,nn,na] = size(A_z);         % n : state vector dimension


%==========================================================================
%======================================================================
%==========================================================================
% PROBLEMA Hinf e LMIs


setlmis([]);

% Declaração das variáveis

gama_z = lmivar(1,[1 1]);

[X_z,n_Xz] = lmivar(1,[n 1]);
[L_z,n_Lz] = lmivar(2,[m n]);

%======================================================================
% Restrições LMIs: norma Hinf
%

j = 1;

for i=1:na
    for k=1:nb
        lmiterm([j 1 1 X_z],A_z(:,:,i),1,'s');
        lmiterm([j 1 1 L_z],B2_z(:,:,k),1,'s');
        lmiterm([j 2 1 0],B1_z');
        lmiterm([j 2 2 0],-1);
        lmiterm([j 3 1 X_z],C1_z,1);
        lmiterm([j 3 1 L_z],D12_z,1);
        lmiterm([j 3 2 0],D11_z);
        lmiterm([j 3 3 gama_z],-1,1)
        j=j+1;
    end
end

%======================================================================
% Restrições de alocação

if aloc_z==1
    
    % LMIs de alocação em uma faixa vertical
    j = j+1;
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_z],A_z(:,:,i),1,'s');
            lmiterm([j 1 1 L_z],B2_z(:,:,k),1,'s');
            lmiterm([j 1 1 X_z],2*alfa_z,1);
            j=j+1;
        end
    end


    j = j+1;
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_z],-A_z(:,:,i),1,'s');
            lmiterm([j 1 1 L_z],-B2_z(:,:,k),1,'s');
            lmiterm([j 1 1 X_z],-2*bet_z,1);
            j=j+1;
        end
    end

    
    j = j+1;
    
    %LMI de alocação no setor cônico
    
    for i=1:na
        for k=1:nb
            lmiterm([j 1 1 X_z],sin(teta_z)*A_z(:,:,i),1,'s');
            lmiterm([j 1 1 L_z],sin(teta_z)*B2_z(:,:,k),1,'s');
            lmiterm([j 2 1 X_z],-cos(teta_z)*A_z(:,:,i),1);
            lmiterm([j 2 1 X_z],1,cos(teta_z)*A_z(:,:,i)');
            lmiterm([j 2 1 L_z],-cos(teta_z)*B2_z(:,:,k),1);
            lmiterm([j 2 1 -L_z],1,cos(teta_z)*B2_z(:,:,k)');
            lmiterm([j 2 2 X_z],sin(teta_z)*A_z(:,:,i),1,'s');
            lmiterm([j 2 2 L_z],sin(teta_z)*B2_z(:,:,k),1,'s');
            j=j+1;
        end
    end
    
    % LMI de alocação no círculo
    
    %     j = j+1;
    %     lmiterm([j 1 1 W1],A(:,:,1),1,'s');
    %     lmiterm([j 1 1 W2],B2(:,:,1),1,'s');
    %     lmiterm([j 1 1 W1],2*alfa,1);
    %     lmiterm([j 1 2 W1],A(:,:,1),1);
    %     lmiterm([j 1 2 W2],B2(:,:,1),1);
    %     lmiterm([j 1 2 W1],alfa,1);
    %     lmiterm([j 2 2 W1],-r,1);
    
end
%======================================================================
% GAMA > 0

j = j+1;
lmiterm([-j 1 1 gama_z],1,1);

%======================================================================

j = j+1;

lmiterm([-j 1 1 X_z],1,1);

%======================================================================

Controlador_Hinf_z = getlmis;


%======================================================================
%======================================================================
% Análise da norma Hinf

%c = mat2dec(LQR_lmi,0,0,0,1);
c=[1 zeros(1,n_Lz-1)]';

options = [1e-8,3000,1e9,1e-4,1];
[c_optz,x_optz] = mincx(Controlador_Hinf_z,c, options);


gama_optz = dec2mat(Controlador_Hinf_z,x_optz,gama_z);
X_optz   = dec2mat(Controlador_Hinf_z,x_optz,X_z);
L_optz   = dec2mat(Controlador_Hinf_z,x_optz,L_z);


% Critério

disp('Valor do norma Hinf via LMI:')

sqrt(gama_optz)


disp('Controlador Hinf via LMI')
Ka_z = L_optz*inv(X_optz);



Kz = Ka_z(1,1:2)
Kiz = Ka_z(1,3)


%=================================

%sim('drone_nao_linear_V3_MF_LQR')
sim('drone_nao_linear_V3_MF_LQR8')
graficos
