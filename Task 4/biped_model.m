pkg load control;
clear all;

Mb = 0.954;
Mw = 0.045;
Jb = 4.134e-3;
Jw = 2.3765e-5;
r = 3.265e-2;
L = 0.07;
Ke = 0.383;
Km = 12.1380;
R = 174.74;
b = 0.002;
g = 9.81;

function [alpha,beta,gamma,delta,epsi] = cart_pendulum_dynamics(Mb,Mw,Jb,Jw,r,L,Ke,Km,R,b,g)
  alpha = (2*((R*b) - (Ke*Km))*(Mb*L*L + Mb*r*L + Jb))/(R*(2*(Jb*Jw + Jw*L*L*Mb + Jb*Mw*r*r +L*L*Mb*Mw*r*r) + Jb*Mb*r*r));
  beta = (-L*L*Mb*Mb*g*r*r)/(Jb*(2*Jw + Mb*r*r + 2*Mw*r*r) +2*Jw*L*L*Mb + 2*L*L*Mb*Mw*r*r);
  gamma = (-2*(R*b - Ke*Km)*(2*Jw +Mb*r*r + 2*Mw*r*r +L*Mb*r))/((r*R*(2*(Jb*Jw + Jw*L*L*Mb + Jb*Mw*r*r +L*L*Mb*Mw*r*r) + Jb*Mb*r*r)));
  delta = (L*Mb*g*(2*Jw + Mb*r*r + 2*Mw*r*r))/(2*Jb*Jw + 2*Jw*L*L*Mb + Jb*Mb*r*r + 2*Jb*Mw*r*r + 2*L*L*Mb*Mw*r*r);
  epsi = Km*r/(R*b - Ke*Km);
endfunction

function [A, B, C, D] = biped(alpha,beta,gamma,delta,epsi,r)
  A = [0 1 0 0 ;
       0 alpha beta -r*alpha; 
       0 0 0 1; 
       0 gamma delta -r*gamma];
  B = [0 ; alpha*epsi ; 0 ; gamma*epsi]; 
  C = [0 0 1 0; 0 0 0 1];
  D = [0; 0]; 
endfunction

function lqr_biped(Mb,Mw,Jb,Jw,r,L,Ke,Km,R,b,g)
  [alpha,beta,gamma,delta,epsi] = cart_pendulum_dynamics(Mb,Mw,Jb,Jw,r,L,Ke,Km,R,b,g);
  [A,B,C,D]=biped(alpha,beta,gamma,delta,epsi,r);   
  sys = ss(A,B,C,D);
  sysd = c2d(sys,10/1000);
  
  Q = [90 0 0 0;
      0 55 0 0;
      0 0 245 0;
      0 0 0 45];
  R = 1;

  K = dlqr(sysd,Q,R);
  printf("\n%d,%d,%d,%d\n",K(1),K(2),K(3),K(4))
  
endfunction
lqr_biped(Mb,Mw,Jb,Jw,r,L,Ke,Km,R,b,g);