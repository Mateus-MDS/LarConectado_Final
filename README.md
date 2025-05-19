LarConectado
Automa��o Residencial com Raspberry Pi Pico W

Descri��o
LarConectado � um sistema de automa��o residencial baseado na Raspberry Pi Pico W que permite o controle de LEDs, exibi��o de temperatura, e monitoramento de sensores por meio de uma interface web, sem uso de bibliotecas externas, utilizando a pilha TCP/IP lwIP diretamente. Por fim, temos o ativamento e desativamento do alarme remotamente e manualmente, com emiss�o da sirene de alarme em caso de movimento.

O sistema tamb�m inclui um display OLED SSD1306, sensores de ambiente como ultrass�nico e LDR, uma matriz de LEDs 5x5 via PIO, e monitora a temperatura interna do chip RP2040.

Objetivos
Controlar LEDs remotamente por p�gina web.

Exibir a temperatura interna da Pico via navegador.

Monitorar ilumina��o ambiente (LDR) e presen�a (ultrass�nico).

Representar informa��es visuais com matriz de LEDs 5x5.

Exibir status e dados no OLED SSD1306.

Prover interface web responsiva sem uso de bibliotecas externas.

Rodar servidor TCP/IP com lwIP direto na Pico W.

Ativa alarme remotamente e manualmente, com acionamento do alarme em casa abertura das portas ou dete��o de pessoas proximas.

Componentes Utilizados
Componente	Pinos	Fun��o

Raspberry Pi Pico W	-	Microcontrolador com Wi-Fi
LED (x2 ou mais) - GPIOs livres (ex: GP2, GP3) - Atuadores de sa�da controlados via web
Display OLED SSD1306 - I�C - GP14 (SDA), GP15 (SCL) - Exibi��o local de dados
Sensor Ultrass�nico HC-SR04	GP16 (TRIG), GP17 (ECHO) - Detec��o de objeto proximo
Sensor LDR (com divisor) - GP26 (ADC0) - Detec��o de luz ambiente
Matriz de LEDs 5�5	PIO - GP7	Visualiza��o gr�fica de estados
Buzzer - GP21 - Emite o som do alarme
Bot�o A - GP5 - Faz o ativamento e desativamento manual do alarme.
Interface Web Wi-Fi - Controle e visualiza��o remota

Funcionamento
Inicializa��o
Configura��o de GPIOs, ADC, PIO e perif�ricos.

Inicializa��o do display OLED.

Conex�o � rede Wi-Fi.

Inicializa��o da pilha lwIP para servi�o TCP/IP.

Upload e parsing do HTML embutido para servidor web.

Interface Web
P�gina HTML simples acess�vel via IP da Pico.

Exibe:

Temperatura interna (atualizada dinamicamente).

Bot�es para acender/apagar LEDs.

Bot�o para ligar/desligar alarme.

Comunica��o por sockets TCP/HTTP diretos via lwIP.

Leitura e Monitoramento
Sensor Ultrass�nico: dist�ncia medida periodicamente.

Sensor LDR: leitura anal�gica para avaliar luminosidade.

Temperatura Interna: leitura do sensor t�rmico do RP2040.

Estrutura do C�digo
Wi-Fi & lwIP Setup

Inicializa interface de rede.

Configura IP fixo ou DHCP.

Inicia servidor HTTP b�sico.

Drivers

OLED SSD1306 via I�C.

Matriz de LEDs via PIO.

Leitura ADC e sensores digitais.

Web Server

Serve HTML est�tico com rotas HTTP simples.

Mapeia comandos de bot�es para GPIOs.

Main Loop

Atualiza sensores.

Atualiza OLED e LED matrix.

Atualiza Alarme.

Processa requisi��es HTTP.

Como Executar o Projeto
Monte os componentes conforme a tabela de pinos.

Configure suas credenciais de Wi-Fi no c�digo.

Compile o firmware com suporte � pilha lwIP (sem RTOS).

Grave o firmware na Pico W.

Acesse o IP atribu�do � Pico na sua rede local pelo navegador.

Use a interface para controlar LEDs e visualizar dados ao vivo.

Requisitos
Raspberry Pi Pico W

SDK do Raspberry Pi Pico configurado

lwIP (sem RTOS, modo standalone)

Display OLED compat�vel com SSD1306

Sensor LDR, Ultrass�nico HC-SR04, LEDs

Reposit�rio
GitHub: github.com/Mateus-MDS/LarConectado

Autor
Nome: Mateus Moreira da Silva


