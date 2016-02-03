%Leitura Serial - Jarvis1

clear Angle Speed Gyro Output Tempo;

close all;

O0 = -147.3;
O1 = 132.3;
O2 = 0;
V0 = 0;
V1 = 1;
V2 = 0;
Kd = - 3.5735;

Tempo_test = 15; %Segundos

s = serial('COM5','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 30, 'Terminator', 'CR/LF');
fopen(s);
t = 0;
i = 1;

str_O0 = sprintf('O0%.3f',O0);
str_O1 = sprintf('O1%.3f',O1);
str_O2 = sprintf('O2%.3f',O2);
str_V0 = sprintf('V0%.3f',V0);
str_V1 = sprintf('V1%.3f',V1);
str_V2 = sprintf('V2%.3f',V2);
str_Kd = sprintf('K%.3f',Kd);
str_t = sprintf('t%u',Tempo_test);

fprintf(s,'%s\r',str_t);
fprintf(s,'%s\r',str_O0);
fprintf(s,'%s\r',str_O1);
fprintf(s,'%s\r',str_O2);
fprintf(s,'%s\r',str_V0);
fprintf(s,'%s\r',str_V1);
fprintf(s,'%s\r',str_V2);
fprintf(s,'%s\r',str_Kd);

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
        Tempo(i) = str2double(out(find(4)+1:tamanho(2)-1));
        i = i +1;
    end
end
fclose(s);
delete(s);

figure;
plot(Angle,'-*b');
title('Angle');
grid on;

figure;
plot(Speed,'-*r');
title('Speed');
grid on;

figure;
plot(Gyro,'-*c');
title('Gyro');
grid on;

figure;
plot(Output,'-*k');
title('Output PID');
grid on;

figure;
plot(Tempo,'-*k');
title('Ticks');
grid on;

figure;
plot(Angle*5,'-*b');
hold on;
plot(Speed/1000,'-*r');
hold on;
plot(Output/1000,'-*k');
title('Todos');
grid on;