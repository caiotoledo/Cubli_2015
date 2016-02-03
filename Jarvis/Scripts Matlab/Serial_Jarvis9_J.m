%Leitura Serial - Jarvis9_J

clear Angle Speed Gyro Output Tempo;

close all;

% Ki = - 1666.7;
% Kt = 0.5;
% Kp = - 17.1538;
% Kd = - 3.5735;

% Kp = -286.8381;
% Ki = -2111.1;
% Kd = -9.3424;
% Kt = 0.5;

Kp = -100;
Ki = 30;
Kd = 15;
Kt = 2;

% Kp = 0;
% Ki = 0;
% Kd = 0;
% Kt = 0;

sp = 0.0261799;

Tempo_test = 30; %Segundos

s = serial('COM3','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 10, 'Terminator', 'CR/LF');
fopen(s);
t = 0;
i = 1;

str_Kd = sprintf('Kd%.3f',Kd);
str_Ki = sprintf('Ki%.3f',Ki);
str_Kt = sprintf('Kt%.3f',Kt);
str_Kp = sprintf('Kp%.3f',Kp);
str_t = sprintf('t%u',Tempo_test);
str_sp = sprintf('sp%.5f',sp);

fprintf(s,'%s\r',str_t);
fprintf(s,'%s\r',str_Kd);
fprintf(s,'%s\r',str_Ki);
fprintf(s,'%s\r',str_Kt);
fprintf(s,'%s\r',str_Kp);
fprintf(s,'%s\r',str_sp);

fprintf(s,'%s\r','go');

while t == 0
    out = fscanf(s);
    t = strcmp(out(1:4),'STOP');
    if t == 0
        find = strfind(out, ' ');
        tamanho = size(out);
        Angle(i) = str2double(out(1:find(1)-1));
        Speed(i) = str2double(out(find(1)+1:find(2)-1));
        Gyro(i) = str2double(out(find(2)+1:find(3)-1));
        Output(i) = str2double(out(find(3)+1:find(4)-1));
        Ticks(i) = str2double(out(find(4)+1:tamanho(2)-1));
        i = i +1;
    end
end
fclose(s);
delete(s);

Tempo = linspace(0,Tempo_test,length(Angle));

figure;
plot(Tempo, (Angle*(180/pi)) ,'-*b');
title('Angle');
grid on;

figure;
plot(Tempo,Speed,'-*r');
title('Speed');
grid on;

figure;
plot(Tempo,Gyro,'-*c');
title('Gyro');
grid on;

figure;
plot(Tempo,Output,'-*k');
title('Output PID');
grid on;

figure;
plot(Ticks,'-*k');
title('Ticks');
grid on;

figure;
plot(Tempo,Angle*5,'-*b');
hold on;
plot(Tempo,Speed/1000,'-*r');
hold on;
plot(Tempo,Output/1000,'-*k');
title('Todos');
grid on;