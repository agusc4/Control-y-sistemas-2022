close all, clc, clear

%% Variables

m=2000;
k=2e6;
c=1*10^(3.5);
a=0.0032;
T=0.001;

%% Modelo de la planta
A=[0 1 0;
   -k/m -c/m +k/m;
   0 0 0];
B=[0;c/(a*m);1/a];
C=[1 0 0];
N=[0;0;0];
D=0;
sys=ss(A,B,C,D);
%% Calculo controlabilidad y observabilidad
W=[B A*B A^2*B];
det(W);
rank(W);

Wo=[C;C*A;C*A^2]; %observable desde la posicion de la carga
det(Wo);
rank(Wo);

%% Calculo K.
% Ecuacion de ricatti A' S + S A −( S B + N) * R^-1 (B' S+N') + Q = 0
Qi1=eye(4); % En funcion del valor estamos penalizando cada una de las variables de estado
Ri1=1; % Se le da una penalizacion a la accion de control

[K1,S1,e1]=lqi(sys,Qi1,Ri1);

%segundo metodo

dx1M=2.25;
dxm1=0;

dx2M=0.65;
dxm2=0;

dx3M=1.5;
dxm3=0;

Qi2=[1 0 0 0 
    0 .01 0 0
    0 0 1 0
    0 0 0 10];
Ri2= 10;       

[K2,S2,e2]=lqi(sys,Qi2,Ri2);

%tercer metodo

Qi3=[.001 0 0 0 
    0 .001 0 0
    0 0 .001 0
    0 0 0 1000];
Ri3=10;



[K3,S3,e3]=lqi(sys,Qi3,Ri3);

K=K3;

%% Ganancias Observador
Q=3e4; % covariance of process and sensor noises, Rv alto el diseñador no tiene confianza en su modelo y considera que tiene ruido, Rw alto considera que sus sensores tienen ruido
R=0.01;
[kest,L,P]=kalman(sys,Q,R,[]);






