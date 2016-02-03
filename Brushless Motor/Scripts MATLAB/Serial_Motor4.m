% Teste 1: Rampa
% Teste 2: Step Variavel
% Teste 3: Step unico
% Teste 4: DAC Receive

%Inicialização de Variaveis:
teste = 2;          %Escolha do Teste: 1 - 4
vel = 0;            %Utilizado: Teste 1, 2 e 3
rampa = 2;          %Tempo de Rampa - Utilizado: Teste 1
constante = 2;      %Utilizado: Teste 1, 2 e 3
div = 1;            %Utilizado: Teste 2
passo_step = 500;   %Utilizado: Teste 2
Ts = 0.01;          %Utilizado: Teste 4 (mas não enviado)

%Leitura da Porta Serial para testes com o Motor:
clear DAC DUTY FREQ;
s = serial('COM3','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 15);
fopen(s);
str_v = sprintf('s%0.1f',vel);
str_r = sprintf('r%0.3f',rampa);
str_c = sprintf('c%0.3f',constante);
str_t = sprintf('t%u',teste);
str_d = sprintf('d%u',div);
str_p = sprintf('p%0.3f',passo_step);
fprintf(s,'%s\r',str_v);
fprintf(s,'%s\r',str_r);
fprintf(s,'%s\r',str_c);
fprintf(s,'%s\r',str_t);
fprintf(s,'%s\r',str_d);
fprintf(s,'%s\r',str_p);
fprintf(s,'%s\r','go');
t = 0;
i = 1;

if teste == 4
    while t == 0
        out = fscanf(s);
        t = strcmp(out(1:2),'GO');
    end
    len = length(motor_data);
    for n = 1:len
        str_dac = sprintf('D%.2f',motor_data(n));
        fprintf(s,'%s\r',str_dac);
        out = fscanf(s);
        find = strfind(out, ' ');
        tamanho = size(out);
        DAC(i) = str2double(out(1:find(1)-1));
        FREQ(i) = str2double(out(find(1)+1:find(2)-1));
        DUTY(i) = str2double(out(find(1)+1:find(3)-1));
        ANGLE(i) = str2double(out(find(3)+1:tamanho(2)-1));
        %pause(Ts);
    end
    fprintf(s,'%s\r','STOP');
else
    while t == 0
        out = fscanf(s);
        t = strcmp(out(1:4),'STOP');
        if t == 0
            find = strfind(out, ' ');
            tamanho = size(out);
            DAC(i) = str2double(out(1:find(1)-1));
            FREQ(i) = str2double(out(find(1)+1:find(2)-1));
            DUTY(i) = str2double(out(find(2)+1:tamanho(2)-1));
            pause(0.0001);
            i = i +1;
        end
    end
end
fclose(s);
delete(s);
figure;
plot(DAC,'-*b');
hold on;
plot(FREQ,'-*r');
hold on;
plot(DUTY,'-*k');
grid on;
legend('DAC','FREQ','Duty');
clear s t i out find tamanho;
clear vel rampa constante teste div passo_step;
clear str_v str_r str_c str_t str_d str_p;

flag = 0;
len = length(FREQ);
VEL = zeros(len);
for i = 1:len
    if flag == 0
        VEL(i) = DUTY(i);
    else
        VEL(i) = FREQ(i);
    end
    if (VEL(i) > 500) && (flag == 0)
        flag = 1;
    elseif (VEL(i) < 500) && (flag == 1)
        flag = 0;
    end
end