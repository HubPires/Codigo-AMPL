reset;

#Dados
param c; #
param m; #m=c+1

set C = 1..c;
set C_p = {1, 2, 3, 4, 5, 6, 7, 8, 9};
set N = 0..m;
set No = 0..c;
set Np = 1..m;

param SR = 5; #
param SL = 5; #
param e = 30; #
param M = 9999; #

#param C1 = ?;
#param C2 = ?;

param tau {i in N, j in N};
param tau_p {i in N, j in N};
#param Clientes {};

#param r {i in Np}

# Variáveis de Decisão
var t {j in Np} >= 0;
var t_p {j in Np} >= 0;
var x {i in No, j in Np} binary; #igual a 1 se caminhão vai do nó i ao nó j, com i dif. j
var y {i in No, j in C, k in Np} binary; #igual a 1 se drone vai do nó i ao nó j e retorna ao caminhão no nó k, com i dif. j e j dif. k
var p {i in No, j in C} binary; #igual a 1 se cliente i é visitado antes do cliente j
var u {i in Np} >= 1;

minimize Z: t[m];

# Restrições
subject to R1: t[0] = 0;
subject to R2: t_p[0] = 0;
subject to R3 {j in C}: p[0,j] = 1;
subject to R4 {i in Np}: u[i] <= c+2;
subject to R5 {i in N}: t[i] >= 0;
subject to R6 {i in N}: t_p[i] >= 0;
subject to R7 {j in C}: sum {i in No} x[i,j] + sum {i in No, k in Np} y[i,j,k] = 1; 
subject to R8: sum {j in Np} x[0,j] = 1;
subject to R9: sum {i in No} x[i,c+1] = 1;
subject to R10 {i in C, j in Np}: u[i]-u[j]+1 <= (c+2)*(1-x[i,j]);
subject to R11 {j in C}: sum {i in No} x[i,j] = sum {k in Np} x[j,k];
subject to R12 {i in No}: sum {j in C, k in Np} y[i,j,k] <= 1; 
subject to R13 {k in Np}: sum {i in No, j in C} y[i,j,k] <= 1; 
subject to R14 {i in C, j in C, k in Np}: 2*y[i,j,k] <= sum {h in No} x[h,i]+sum {l in C} x[l,k];
subject to R15 {j in C, k in Np}: y[0,j,k] <= sum {h in No} x[h,k];
subject to R16 {i in C, k in Np}: u[k]-u[i] >= 1-(c+2)*(1-sum {j in C} y[i,j,k]);
subject to R17 {i in C}: t_p[i] >= t[i]-M*(1-sum {j in C, k in Np} y[i,j,k]);
subject to R18 {i in C}: t_p[i] <= t[i]+M*(1-sum {j in C, k in Np} y[i,j,k]);
subject to R19 {k in Np}: t_p[k] >= t[k]-M*(1-sum {i in No, j in C} y[i,j,k]);
subject to R20 {k in Np}: t_p[k] <= t[k]+M*(1-sum {i in No, j in C} y[i,j,k]);
subject to R21 {h in No, k in Np}: t[k] >= t[h]+tau[h,k]+SL*(sum {l in C, b in Np} y[k,l,b])+SR*(sum {i in No, j in C} y[i,j,k])-M*(1-x[h,k]);
subject to R22 {j in C_p, i in No}: t_p[j] >= t_p[i]+tau_p[i,j]-M*(1-sum {k in Np} y[i,j,k]);
subject to R23 {j in C_p, k in Np}: t_p[k] >= t_p[j]+tau_p[j,k]+SR-M*(1-sum {i in No} y[i,j,k]);
subject to R24 {k in Np, j in C, i in No}: t_p[k]-(t_p[j]-tau_p[i,j]) <= e + M*(1-y[i,j,k]);
subject to R25 {i in C, j in C}: u[i]-u[j] >= 1-(c+2)*p[i,j];
subject to R26 {i in C, j in C}: u[i]-u[j] <= -1+(c+2)*(1-p[i,j]);
subject to R27 {i in C, j in C}: p[i,j]+p[j,i] = 1;
subject to R28 {i in No, k in Np, l in C}: t_p[l] >= t_p[k]-M*(3-sum {j in C} y[i,j,k]-sum {d in C, n in Np} y[l,d,n]-p[i,l]);

