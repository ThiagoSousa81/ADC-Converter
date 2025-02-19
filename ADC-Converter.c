/*
 * Por: Wilton Lacerda Silva
 *    Interface Homem-Máquina (IHM) com o Display OLED
 * 
 * Este programa utiliza o display OLED SSD1306 com resolução de 128x64 pixels
 * e o microcontrolador RP2040 (Raspberry Pi Pico) para exibir informações
 * do conversor analógico-digital (ADC) e do Joystick.
 * Também mostra a leitura dos botões do Joystick e do botão A.
 * 
 * 
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22 // GPIO para botão do Joystick
#define Botao_A 5 // GPIO para botão A

#define LED_R 13 // GPIO para LED Vermelho
#define LED_G 11 // GPIO para LED Verde
#define LED_B 12 // GPIO para LED Azul

// Tempo para debounce
static volatile uint32_t last_time = 0;

// Estado do LED verde
bool led_green_state = false;

// Estado do PWM
bool state_PWM = true;

// Atualização das constantes...
#define PWM_FREQ 50        // Frequência do sinal de PWM em 50 Hz
#define CLK_DIV 100.0f     // Divisor de clock escolhido
#define WRAP_VALUE 24999   // (125.000.000Hz / (100 * 50Hz)) - 1 = 24999

void gpio_irq_handler(uint gpio, uint32_t events)
{
  // Obtém o tempo atual em microssegundos
  uint32_t current_time = to_us_since_boot(get_absolute_time());
  // Verifica se passou tempo suficiente desde o último evento
  if (current_time - last_time > 200000) // 200 ms de debouncing
  {
    if (gpio == JOYSTICK_PB)
    {
      // Altera o estado do LED verde      
      led_green_state = !led_green_state;
      //printf("Estado do LED: %s\n", led_green_state ? "ON" : "OFF");
      gpio_put(LED_G, led_green_state);      
      
    }
    else if (gpio == Botao_A)
    {
      state_PWM = !state_PWM;
    }
    last_time = current_time; // Atualiza o tempo do último evento
  }
}

// Função de conversão
uint32_t us_to_level(uint32_t us) {
  // Cada unidade = (20ms / 25000) = 0.8µs
  return (uint32_t)(us * 1.25f); // 1µs = 1.25 unidades
}

int main()
{
  stdio_init_all();
  set_sys_clock_khz(125000, true);  // Clock em 125 MHz

  gpio_init(LED_G);
  gpio_set_dir(LED_G, GPIO_OUT);  

  gpio_set_function(LED_R, GPIO_FUNC_PWM);
  gpio_set_function(LED_B, GPIO_FUNC_PWM);
  
  uint slice_num_R = pwm_gpio_to_slice_num(LED_R);
  uint slice_num_B = pwm_gpio_to_slice_num(LED_B);

  uint channel_R = pwm_gpio_to_channel(LED_R);
  uint channel_B = pwm_gpio_to_channel(LED_B);

  // Configura e inicia o PWM 
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, CLK_DIV);
  pwm_config_set_wrap(&config, WRAP_VALUE);

  pwm_init(slice_num_R, &config, true);
  pwm_init(slice_num_B, &config, true);



  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB); 
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  gpio_init(Botao_A);
  gpio_set_dir(Botao_A, GPIO_IN);
  gpio_pull_up(Botao_A);  

  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);  
  


  uint16_t adc_value_x;
  uint16_t adc_value_y;  
  char str_x[5];  // Buffer para armazenar a string
  char str_y[5];  // Buffer para armazenar a string  
  
  bool cor = true;
  while (true)
  {
    adc_select_input(0); // Seleciona o ADC para eixo X. O pino 26 como entrada analógica
    adc_value_x = adc_read();
    adc_select_input(1); // Seleciona o ADC para eixo Y. O pino 27 como entrada analógica
    adc_value_y = adc_read();    
    sprintf(str_x, "%d", adc_value_x);  // Converte o inteiro em string
    sprintf(str_y, "%d", adc_value_y);  // Converte o inteiro em string
    
    //cor = !cor;
    // Atualiza o conteúdo do display com animações
    ssd1306_fill(&ssd, !cor); // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Desenha um retângulo
    ssd1306_line(&ssd, 3, 25, 123, 25, cor); // Desenha uma linha
    ssd1306_line(&ssd, 3, 37, 123, 37, cor); // Desenha uma linha   
    ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6); // Desenha uma string
    ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16); // Desenha uma string
    ssd1306_draw_string(&ssd, "ADC   JOYSTICK", 10, 28); // Desenha uma string 
    ssd1306_draw_string(&ssd, "X    Y    PB", 20, 41); // Desenha uma string    
    ssd1306_line(&ssd, 44, 37, 44, 60, cor); // Desenha uma linha vertical         
    ssd1306_draw_string(&ssd, str_x, 8, 52); // Desenha uma string     
    ssd1306_line(&ssd, 84, 37, 84, 60, cor); // Desenha uma linha vertical      
    ssd1306_draw_string(&ssd, str_y, 49, 52); // Desenha uma string   
    ssd1306_rect(&ssd, 52, 90, 8, 8, cor, !gpio_get(JOYSTICK_PB)); // Desenha um retângulo  
    ssd1306_rect(&ssd, 52, 102, 8, 8, cor, !gpio_get(Botao_A)); // Desenha um retângulo    
    ssd1306_rect(&ssd, 52, 114, 8, 8, cor, !cor); // Desenha um retângulo       
    ssd1306_send_data(&ssd); // Atualiza o display

    if (!(adc_value_x > 4079) || !(adc_value_x < 31))
    {
      pwm_set_chan_level(slice_num_R, channel_R, us_to_level(fabs(adc_value_x)));
    }
    else
    {
      pwm_set_chan_level(slice_num_R, channel_R, us_to_level(24999));
    }

    if (!(adc_value_y > 4079) || !(adc_value_y < 31))
    {
      pwm_set_chan_level(slice_num_B, channel_B, us_to_level(fabs(adc_value_x)));
    }
    else
    {
      pwm_set_chan_level(slice_num_B, channel_B, us_to_level(24999));
    }
    



    sleep_ms(100);
  }
}