/*
 * Sistema de Controle Residencial Inteligente para Raspberry Pi Pico
 *
 * Este programa implementa um sistema de automação residencial controlado via web com:
 * - Conexão WiFi para controle remoto
 * - Matriz de LEDs para exibição de status
 * - Display OLED para notificações
 * - Sensor ultrassônico para detecção de proximidade
 * - Sensor de luz para iluminação automática
 * - Monitoramento de temperatura
 * - Interface web para controle de dispositivos domésticos
 * - Alarme ativado manualmente e remotamente
 */

#include <stdio.h>               // Funções padrão de entrada/saída
#include <string.h>              // Funções para manipulação de strings
#include <stdlib.h>              // Alocação de memória e outras utilidades

#include "pico/stdlib.h"         // Funções padrão do Raspberry Pi Pico
#include "hardware/adc.h"        // Funções do ADC (Conversor Analógico-Digital)
#include "pico/cyw43_arch.h"     // Driver WiFi CYW43

#include "lwip/pbuf.h"           // Manipulação de buffers de pacotes IP
#include "lwip/tcp.h"            // Implementação do protocolo TCP
#include "lwip/netif.h"          // Funções de interface de rede

#include "hardware/i2c.h"        // Interface I2C
#include "inc/ssd1306.h"         // Driver para display OLED
#include "inc/font.h"            // Definições de fontes para o display
#include "hardware/pio.h"        // Funções de I/O programável
#include "hardware/clocks.h"     // Funções de controle de clock
#include "animacoes_led.pio.h"   // Programa PIO para animações de LED

// Credenciais da rede WiFi - Cuidado ao compartilhar publicamente!
#define WIFI_SSID "******"
#define WIFI_PASSWORD "********"

/* ========== DEFINIÇÕES DE HARDWARE ========== */

// Configuração da matriz de LEDs
#define NUM_PIXELS 25           // Número de LEDs na matriz
#define matriz_leds 7           // Pino de saída para a matriz

// Configuração I2C para o display OLED
#define I2C_PORT i2c1           // Porta I2C utilizada
#define I2C_SDA 14              // Pino SDA
#define I2C_SCL 15              // Pino SCL
#define ENDERECO 0x3C           // Endereço I2C do display
#define WIDTH 128               // Largura do display em pixels
#define HEIGHT 64               // Altura do display em pixels

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN  // GPIO do chip CYW43
#define LED_BLUE_PIN 12                // GPIO12 - LED azul
#define LED_GREEN_PIN 11               // GPIO11 - LED verde
#define LED_RED_PIN 13                 // GPIO13 - LED vermelho

// Pinos para o sensor ultrassônico
#define TRIG_PIN 8              // Pino de trigger do sensor
#define ECHO_PIN 9              // Pino de echo do sensor


// Pinos para o sensor ultrassônico do Alarme
#define TRIG_PIN_2 18              // Pino de trigger do sensor
#define ECHO_PIN_2 19              // Pino de echo do sensor

#define BUZZER 21                  // Pino do buzzer

// Pino para o sensor de luz (LDR)
#define ldr_pin 16

// Pinos de entrada do joystick
#define ADC_JOYSTICK_X 26  // Pino ADC para eixo X
#define ADC_JOYSTICK_Y 27  // Pino ADC para eixo Y

#define Botao_A 5          // pino do botão A

// Variáveis globais para controle dos dispositivos
PIO pio;                       // Controlador PIO
uint sm;                       // State Machine do PIO
uint contagem = 5;             // Contador para exibição na matriz
ssd1306_t ssd;                 // Estrutura do display OLED

int tv = 0; int tv_alarme = 0;
uint Eixo_x_value, Eixo_Y_value;

// Estados dos dispositivos (ligado/desligado)
bool estado_led_sala = false;
bool estado_led_cozinha = false;
bool estado_led_quarto = false;
bool estado_led_banheiro = false;
bool estado_led_quintal = false;
bool estado_display = false;
bool estado_alarme = false;
bool Alarme_Acionado = false;

/* ========== PROTÓTIPOS DE FUNÇÕES ========== */
void gpio_led_bitdog(void);    // Inicializa os GPIOs dos LEDs
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err); // Callback para conexões TCP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); // Callback para recebimento de dados
float temp_read(void);         // Lê a temperatura interna
void user_request(char **request); // Processa as requisições do usuário
void ligar_luz();              // Controla a matriz de LEDs
void ligar_display();          // Controla o display OLED
void send_trigger_pulse();     // Envia pulso para o sensor ultrassônico
float measure_distance_cm();   // Mede a distância com o sensor ultrassônico
void luz_frente_controlada();  // Controla os LEDs frontais baseado em sensores
void Alarme();
void Som_Alarme();
void gpio_irq_handler(uint gpio, uint32_t events);

/* ========== IMPLEMENTAÇÃO DAS FUNÇÕES ========== */

// Função principal
int main() {
    // Inicializa todas as bibliotecas padrão
    stdio_init_all();

    // Inicializa os GPIOs dos LEDs
    gpio_led_bitdog();

    // Configuração do PIO para a matriz de LEDs
    pio = pio0;
    uint offset = pio_add_program(pio, &animacoes_led_program);
    sm = pio_claim_unused_sm(pio, true);
    animacoes_led_program_init(pio, sm, offset, matriz_leds);

    // Configuração do I2C para o display OLED
    i2c_init(I2C_PORT, 400 * 1000);  // Inicializa I2C a 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Inicialização do display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);  // Limpa o display
    ssd1306_send_data(&ssd);

    // Inicializa a interrupção no botão A
    gpio_set_irq_enabled_with_callback(Botao_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa o chip WiFi
    while (cyw43_arch_init()) {
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }

    // Configura o LED do WiFi como desligado inicialmente
    cyw43_arch_gpio_put(LED_PIN, 0);

    // Configura o modo Station para conectar a uma rede WiFi
    cyw43_arch_enable_sta_mode();

    // Tenta conectar ao WiFi
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }
    printf("Conectado ao Wi-Fi\n");

    // Exibe o IP atribuído ao dispositivo
    if (netif_default) {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Configura o servidor TCP na porta 80 (HTTP)
    struct tcp_pcb *server = tcp_new();
    if (!server) {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    // Associa o servidor à porta 80
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca o servidor em modo de escuta
    server = tcp_listen(server);
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    // Inicializa o ADC para leitura de temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // Loop principal do programa
    while (true) {
        // Controla os LEDs frontais baseado nos sensores
        luz_frente_controlada();
        
        // Atualiza o estado da matriz de LEDs
        ligar_luz();
        
        // Atualiza o display OLED
        ligar_display();

        // Verifica o alarme, e aciona se necessário
        Alarme();
        
        // Processa eventos de rede
        cyw43_arch_poll();
        
        // Pequena pausa para reduzir uso da CPU
        sleep_ms(100);
    }

    // Desliga o WiFi antes de encerrar
    cyw43_arch_deinit();
    return 0;
}

/* ========== FUNÇÕES DE HARDWARE ========== */

// Inicializa os pinos GPIO para os LEDs e sensores
void gpio_led_bitdog(void) {
    // Configura os LEDs RGB como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);

    // Configura o sensor ultrassônico do Led RGB
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Configura o sensor ultrassônico do Alarme
    gpio_init(TRIG_PIN_2);
    gpio_set_dir(TRIG_PIN_2, GPIO_OUT);
    gpio_put(TRIG_PIN_2, 0);

    gpio_init(ECHO_PIN_2);
    gpio_set_dir(ECHO_PIN_2, GPIO_IN);

    // Inicialização do Buzzer
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);

    // Configura o sensor de luz (LDR)
    gpio_init(ldr_pin);
    gpio_set_dir(ldr_pin, GPIO_IN);

    // Configuração ADC
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_gpio_init(ADC_JOYSTICK_Y);

    // Configuração GPIO
    gpio_init(Botao_A);
    gpio_set_dir(Botao_A, GPIO_IN);
    gpio_pull_up(Botao_A);
}

/* ========== FUNÇÕES DE CONTROLE ========== */

// Controla a matriz de LEDs baseado nos estados dos cômodos
void ligar_luz() {
    uint32_t luz_sala, luz_cozinha, luz_quarto, luz_banheiro, luz_quintal;

    // Define as cores para cada cômodo baseado no estado
    luz_sala = estado_led_sala ? 0xFFFFFF00 : 0x00000000;
    luz_cozinha = estado_led_cozinha ? 0xFFFFFF00 : 0x00000000;
    luz_quarto = estado_led_quarto ? 0xFFFFFF00 : 0x00000000;
    luz_banheiro = estado_led_banheiro ? 0xFFFFFF00 : 0x00000000;
    luz_quintal = estado_led_quintal ? 0xFFFFFF00 : 0x00000000;

    // Atualiza cada LED da matriz 5x5
    for (int i = 0; i < NUM_PIXELS; i++) {
        uint32_t valor_led = 0;
        int linha = i / 5;  // Divide a matriz em linhas
        
        // Atribui a cor baseado na linha (cômodo)
        if (linha == 4) {
            valor_led = luz_sala;
        } else if (linha == 3) {
            valor_led = luz_cozinha;
        } else if (linha == 2) {
            valor_led = luz_quarto;
        } else if (linha == 1) {
            valor_led = luz_banheiro;
        } else if (linha == 0) {
            valor_led = luz_quintal;
        } else {
            valor_led = 0x000000;  // LED apagado
        }
        
        // Envia o valor para o LED
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Controla o display OLED
void ligar_display() {
    bool cor = true;  // Cor branca para o texto

    // Limpa e desenha a moldura do display
    ssd1306_fill(&ssd, !cor);
    ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna

    // Exibe o estado da TV
    if (estado_display) {
        // Limpa e desenha a moldura do display
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna
        ssd1306_draw_string(&ssd, "TELEVISAO ", 30, 30);
        ssd1306_draw_string(&ssd, "LIGADA", 38, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        tv = 1;
    } else {
        if (tv == 1){
            // Limpa e desenha a moldura do display
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna
        ssd1306_draw_string(&ssd, "TELEVISAO ", 30, 30);
        ssd1306_draw_string(&ssd, "DESLIGADA", 28, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        sleep_ms(2000);

        ssd1306_fill(&ssd, !cor);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        tv = 0; // para evitar multiplos desligamentos
        }
    }

    if (estado_alarme){
        // Limpa e desenha a moldura do display
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna
        ssd1306_draw_string(&ssd, "ALARME", 35, 30);
        ssd1306_draw_string(&ssd, "LIGADO", 35, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        tv_alarme = 1; // Para evitar multiplos desligamento de alarme

        // Caso haja violação e o alarme seja acionado
        if (Alarme_Acionado){
            // Limpa e desenha a moldura do display
            ssd1306_fill(&ssd, !cor);
            ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna
            ssd1306_draw_string(&ssd, "ALARME", 35, 30);    
            ssd1306_draw_string(&ssd, "ACIONADO", 28, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);
        }
    } else {
        if (tv_alarme == 1){
            // Limpa e desenha a moldura do display
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);    // Moldura externa
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);    // Moldura interna
        ssd1306_draw_string(&ssd, "ALARME", 35, 30);
        ssd1306_draw_string(&ssd, "DESLIGADO", 28, 40);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        sleep_ms(2000);

        ssd1306_fill(&ssd, !cor);

        // Atualiza o display
        ssd1306_send_data(&ssd);

        tv_alarme = 0; // para não desligar novamente
        }
    }
}

/* ========== FUNÇÕES DOS SENSORES ========== */

// Envia um pulso para o sensor ultrassônico
void send_trigger_pulse() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}

// Envia um pulso para o sensor ultrassônico do Alarme
void send_trigger_pulse_2() {
    gpio_put(TRIG_PIN_2, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN_2, 0);
}

// Mede a distância usando o sensor ultrassônico genérico
float measure_distance_cm(uint trigger_pin, uint echo_pin) {

    // Envia pulso de trigger
    gpio_put(trigger_pin, 0);
    sleep_us(2);
    gpio_put(trigger_pin, 1);
    sleep_us(10);
    gpio_put(trigger_pin, 0);

    // Espera o pino ECHO ficar em HIGH
    while (gpio_get(echo_pin) == 0);

    absolute_time_t start = get_absolute_time();

    // Espera o pino ECHO voltar para LOW
    while (gpio_get(echo_pin) == 1);

    absolute_time_t end = get_absolute_time();
    int64_t pulse_duration = absolute_time_diff_us(start, end);

    // Converte para centímetros
    float distance_cm = pulse_duration / 58.0;

    return distance_cm;
}


// Controla os LEDs frontais baseado nos sensores
void luz_frente_controlada() {
    float dist1 = measure_distance_cm(TRIG_PIN, ECHO_PIN);
    
    // Aciona os LEDs se houver objeto próximo e estiver escuro
    if ((dist1 < 15) && (!gpio_get(ldr_pin))) {
        gpio_put(LED_BLUE_PIN, 1);
        gpio_put(LED_GREEN_PIN, 1);
        gpio_put(LED_RED_PIN, 1);
    } else {
        gpio_put(LED_BLUE_PIN, 0);
        gpio_put(LED_GREEN_PIN, 0);
        gpio_put(LED_RED_PIN, 0);
    }
}

/**
 * Gera um som de alerta no Buzzer.
 */
void Som_Alarme() {
    const uint buzzer_pin = BUZZER;
    const int freq = 2500;           // Frequência em Hz
    const int duration_ms = 80;      // Duração de cada bip
    const int interval_ms = 50;      // Intervalo entre bipes

    int period_us = 1000000 / freq;  // Período total da onda
    int half_period_us = period_us / 2;

    for (int i = 0; i < 8; i++) {
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        while (to_ms_since_boot(get_absolute_time()) - start_time < duration_ms) {
            gpio_put(buzzer_pin, 1);
            sleep_us(half_period_us);
            gpio_put(buzzer_pin, 0);
            sleep_us(half_period_us);
        }
        sleep_ms(interval_ms);
    }
}

// Função verefica se houve violação em detecção de objetos proximos ou violação das portas, e aciona o alarme
void Alarme(){
    // calcula a distância do objeto ao ultrassônico
    float dist2 = measure_distance_cm(TRIG_PIN_2, ECHO_PIN_2);

    // Leitura dos sensores
    adc_select_input(0);
    Eixo_x_value = adc_read();
    adc_select_input(1);
    Eixo_Y_value = adc_read();

    // Caso o alarme esteja ativado, e houver abertura das portas, há o acionamneto do alarme
    if (estado_alarme){
        if (((Eixo_Y_value > 2200) || (Eixo_Y_value < 1800)) || ((Eixo_x_value > 2200) || (Eixo_x_value < 1800)))
    {
        Alarme_Acionado = true;
    }
    // detecção de objetos proximo ultrassônico resulta no acionamento do alarme    
    if (dist2 < 15){
        Alarme_Acionado = true;
    }

    // chama a funcão que faz atuação do som no buzzer
    if (Alarme_Acionado){
        Som_Alarme();
    }
    }
}

/* ========== HANDLER DE INTERRUPÇÃO ========== */

// Tratamento das interrupções dos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Debouncing de 300ms
    if (current_time - last_time > 300000) {
        last_time = current_time;

        // Botão A pressionado, faz a alternância de estado do botão A
        if (gpio == Botao_A && !gpio_get(Botao_A)) {
            estado_alarme = !estado_alarme;
            if(!estado_alarme){
                Alarme_Acionado = false;
            }
        }
    }
}

/* ========== FUNÇÕES DE REDE ========== */

// Callback para aceitar novas conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Processa as requisições do usuário
void user_request(char **request) {
    // Verifica qual comando foi recebido e altera o estado correspondente
    if (strstr(*request, "GET /mudar_estado_luz_sala") != NULL) {
        estado_led_sala = !estado_led_sala;
    }
    else if (strstr(*request, "GET /mudar_estado_luz_cozinha") != NULL) {
        estado_led_cozinha = !estado_led_cozinha;
    }
    else if (strstr(*request, "GET /mudar_estado_luz_quarto") != NULL) {
        estado_led_quarto = !estado_led_quarto;
    }
    else if (strstr(*request, "GET /mudar_estado_luz_banheiro") != NULL) {
        estado_led_banheiro = !estado_led_banheiro;
    }
    else if (strstr(*request, "GET /mudar_estado_luz_quintal") != NULL) {
        estado_led_quintal = !estado_led_quintal;
    }
    else if (strstr(*request, "GET /mudar_estado_display") != NULL) {
        estado_display = !estado_display;
    }
    else if (strstr(*request, "GET /mudar_estado_alarme") != NULL) {
        estado_alarme = !estado_alarme;
        if(!estado_alarme){
            Alarme_Acionado = false;
        }
    }
    else if (strstr(*request, "GET /on") != NULL) {
        cyw43_arch_gpio_put(LED_PIN, 1);
    }
    else if (strstr(*request, "GET /off") != NULL) {
        cyw43_arch_gpio_put(LED_PIN, 0);
    }
}

// Lê a temperatura interna do RP2040
float temp_read(void) {
    adc_select_input(4);  // Seleciona o canal do sensor de temperatura
    uint16_t raw_value = adc_read();
    
    // Fórmula de conversão para temperatura (documentação do RP2040)
    const float conversion_factor = 3.3f / (1 << 12);
    float temperature = 27.0f - ((raw_value * conversion_factor) - 0.706f) / 0.001721f;
    
    return temperature;
}

// Callback para recebimento de dados TCP (requisições HTTP)
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Copia a requisição para um buffer
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("Request: %s\n", request);

    // Processa a requisição do usuário
    user_request(&request);
    
    // Lê a temperatura atual
    float temperature = temp_read();

    // HTML da página web
    char html[2096];
    snprintf(html, sizeof(html),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "\r\n"
             "<!DOCTYPE html>\n"
             "<html>\n"
             "<head>\n"
             "<title>Controle Residencial</title>\n"
             "<style>\n"
             "body { background-color:rgb(170, 240, 181); font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
             "h1 { font-size: 40px; margin-bottom: 20px; }\n"
             "button { background-color: LightBlue; font-size: 20px; margin: 5px; padding: 10px 20px; border-radius: 10px; }\n"
             ".temperature { font-size: 24px; margin-top: 20px; color: #333; }\n"
             "</style>\n"
             "</head>\n"
             "<body>\n"
             "<h1>Controle Residencial</h1>\n"
             "<form action=\"./mudar_estado_luz_sala\"><button>Luz da Sala</button></form>\n"
             "<form action=\"./mudar_estado_luz_cozinha\"><button>Luz da Cozinha</button></form>\n"
             "<form action=\"./mudar_estado_luz_quarto\"><button>Luz do Quarto</button></form>\n"
             "<form action=\"./mudar_estado_luz_banheiro\"><button>Luz do Banheiro</button></form>\n"
             "<form action=\"./mudar_estado_luz_quintal\"><button>Luz do Quintal</button></form>\n"
             "<form action=\"./mudar_estado_alarme\"><button>Alarme</button></form>\n"
              "<p class=\"temperature\">Temperatura Interna: %.2f &deg;C</p>\n"
             "</body>\n"
             "</html>\n",
             temperature);

    // Envia a resposta HTTP
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Libera a memória alocada
    free(request);
    pbuf_free(p);

    return ERR_OK;
}