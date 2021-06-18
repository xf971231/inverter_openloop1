w = 2 * pi * 50 ;
Q = 0.1 ;

t = tf([1, 0, w^2], [1, w/Q, w^2]) ;
bode(t)

close all;
clc;clear all;
omega_ref = 0.4*pi;

rho_1 = 0.7;
num_Cz_1 = [1, -2*cos(omega_ref), 1];
den_Cz_1 = [1, -2*rho_1*cos(omega_ref),rho_1^2];
[h_Cz_1,w_Cz_1] = freqz(num_Cz_1,den_Cz_1);%Ƶ����Ӧ

rho_2 = 0.8;
num_Cz_2 = [1, -2*cos(omega_ref), 1];
den_Cz_2 = [1, -2*rho_2*cos(omega_ref),rho_2^2];
[h_Cz_2,w_Cz_2] = freqz(num_Cz_2,den_Cz_2);%Ƶ����Ӧ

rho_3 = 0.9;
num_Cz_3 = [1, -2*cos(omega_ref), 1];
den_Cz_3 = [1, -2*rho_3*cos(omega_ref),rho_3^2];
[h_Cz_3,w_Cz_3] = freqz(num_Cz_3,den_Cz_3);%Ƶ����Ӧ

figure(1);
plot(w_Cz_1,abs(h_Cz_1),'r',w_Cz_2,abs(h_Cz_2),'b',w_Cz_3,abs(h_Cz_3),'k');
xlabel('Ƶ��');ylabel('����');
legend('rho=0.7','rho=0.8','rho=0.9','Location','SouthEast');
title('�ݲ��˲����ķ�Ƶ�������ߣ�δ�ӷ������ڣ�');grid;


Cz = filt(num_Cz_3,den_Cz_3); %������ɢ���ݺ���

alpha_1 = 5;
HNz_1 = (1+alpha_1)*Cz/(1+alpha_1*Cz);
[h_HNz_1,w_HNz_1] = freqz(HNz_1.num{1}, HNz_1.den{1});%Ƶ����Ӧ

alpha_2 = 20;
HNz_2 = (1+alpha_2)*Cz/(1+alpha_2*Cz);
[h_HNz_2,w_HNz_2] = freqz(HNz_2.num{1}, HNz_2.den{1});%Ƶ����Ӧ

figure(2);
plot(w_Cz_1,abs(h_Cz_1),'r',w_HNz_1,abs(h_HNz_1),'b',w_HNz_2,abs(h_HNz_2),'k');
xlabel('Ƶ��');ylabel('����');
legend('Cz (rho=0.9)','HNz (alpha=5)','HNz (alpha=20)','Location','SouthEast');
title('�ݲ��˲����ķ�Ƶ��������');grid;

