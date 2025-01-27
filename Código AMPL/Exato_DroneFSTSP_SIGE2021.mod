reset;

#Dados
param c = 10; #
param m = 11; #m=c+1

param tau {i in 0..m,j in 0..m} >= 0;
param tau_p {i in 0..m,j in 0..m} >= 0;

read {i in 0..m,j in 0..m} tau[i,j] <20140810T123440v6/tau.csv;
read {i in 0..m,j in 0..m} tau_p[i,j] <20140810T123440v6/tauprime.csv;

param C_p {1..c} >= 0;
read {i in 1..c} C_p[i] <20140810T123440v6/cprime_2.csv;

set C = 1..c;
set N = 0..m;
set No = 0..c;
set Np = 1..m;

param SR = 1; #
param SL = 1; #
param e = 20; #
param M = 9999; #

set F = {i in No, j in C, k in Np: i != j != k != i >= 0 <= C_p[j] == 0};
set A = {(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),(0,8),(0,9),(0,10),(0,11),
(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),(1,8),(1,9),(1,10),(1,11),(2,1),(2,3),(2,4),
(2,5),(2,6),(2,7),(2,8),(2,9),(2,10),(2,11),(3,1),(3,2),(3,4),(3,5),(3,6),(3,7),
(3,8),(3,9),(3,10),(3,11),(4,1),(4,2),(4,3),(4,5),(4,6),(4,7),(4,8),(4,9),(4,10),
(4,11),(5,1),(5,2),(5,3),(5,4),(5,6),(5,7),(5,8),(5,9),(5,10),(5,11),(6,1),(6,2),
(6,3),(6,4),(6,5),(6,7),(6,8),(6,9),(6,10),(6,11),(7,1),(7,2),(7,3),(7,4),(7,5),
(7,6),(7,8),(7,9),(7,10),(7,11),(8,1),(8,2),(8,3),(8,4),(8,5),(8,6),(8,7),(8,9),
(8,10),(8,11),(9,1),(9,2),(9,3),(9,4),(9,5),(9,6),(9,7),(9,8),(9,10),(9,11),(10,1),
(10,2),(10,3),(10,4),(10,5),(10,6),(10,7),(10,8),(10,9),(10,11)};

# Vari�veis de Decis�o
var x {(i,j) in A} binary; #igual a 1 se caminh�o vai do n� i ao n�, com i dif. j
var y {i in No, j in C, k in Np} binary; #igual a 1 se drone vai do n� i ao n� j e retorna ao caminh�o no n� k, com i dif. j e j dif. k
var t {i in N} >= 0;
var w {i in Np} >= 0;
var z {i in N} binary; #igual a 1 se drone est� no caminh�o no n� i


minimize Z: sum {(i,j) in A} tau[i,j]*x[i,j]+ 
			SR*sum {j in C, k in Np: 0 != j != k != 0 >= 0 <= C_p[j] == 0} y[0,j,k]+
			(SL)*sum {i in No, j in C, k in Np:  0 !=i != j != k != 0 >= 0 <= C_p[j] == 0} y[i,j,k] +
			(SR)*sum {i in No, j in C, k in Np:  0 !=i != j != k != 0 >= 0 <= C_p[j] == 0} y[i,j,k] +
			sum {i in Np} w[i];

# Restri��es
subject to R2 {j in C}: sum {(i,j) in A} x[i,j] + sum {i in No, k in Np: i != j != k != i >= 0 <= C_p[j] == 0} y[i,j,k] = 1; 
subject to R3 {j in C}: sum {(j,i) in A} x[j,i] + sum {i in No, k in Np: i != j != k != i >= 0 <= C_p[j] == 0} y[i,j,k] = 1; 
subject to R41: sum {j in Np} x[0,j] = 1;
subject to R42: sum {i in No} x[i,m] = 1;
subject to R5 {j in C}: sum {(i,j) in A} x[i,j] = sum {(j,i) in A} x[j,i];
subject to R6 {(i,j) in A}: t[j] >= t[i]+tau[i,j]-M*(1-x[i,j]);
subject to R7 {i in No, j in C, k in Np: i != j != k != 0 >= 0 <= C_p[j] == 0}: t[k] >= t[i]+tau_p[i,j]+tau_p[j,k]-M*(1-y[i,j,k]);
subject to R8 {(i,j) in A}: w[j] >= t[j]-t[i]-tau[i,j]-M*(1-x[i,j]);
subject to R9 {(i,k) in A}: t[k]-t[i]+SR-M*(1-sum {j in C: i != j != k != 0 >= 0 <= C_p[j] == 0} y[i,j,k]) <= e;
subject to R10 {i in Np}: z[i] <= sum {(j,i) in A} x[j,i];
subject to R11 {i in No}: sum {j in C, k in Np: i != j != k != 0 >= 0 <= C_p[j] == 0} y[i,j,k] <= z[i];
subject to R11_2 {k in Np}: sum {j in C, i in No: i != j != k != 0 >= 0 <= C_p[j] == 0} y[i,j,k] <= z[k];
subject to R12 {(i,j) in A}: z[j] <= z[i]-x[i,j]+
							 sum {l in No, k in C: l != k != j != 0 >= 0 <= C_p[k] == 0} y[l,k,j]-
							 sum {k in C, l in Np: i != k != l != 0 >= 0 <= C_p[k] == 0} y[i,k,l]+1;
subject to R13: t[0] = 0;