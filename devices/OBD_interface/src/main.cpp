/* ============================================================================

    ESP32 Web Server

    Espressif IDE 2.6.0

    Author: Dr. Eng. Wagner Rambo
    Date:   June, 2024

============================================================================ */


// ============================================================================
// --- libraries ---
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "interface.h"


// ============================================================================
// --- Hardware Mapping ---
#define   LED_STATUS  GPIO_NUM_2                         //se ligado, indica que o ESP32 se conectou ao Wifi
#define   LED1        GPIO_NUM_32                        //LED1 a ser controlado
#define   LED2        GPIO_NUM_33                        //LED2 a ser controlado


// ============================================================================
// --- constants ---
static const char *TAG = "LED_CONTROL";                  //tag para os logs do servidor


// ============================================================================
// --- Macros ---
#define WIFI_SSID "YOUR SSID"                             //insira entre aspas o nome de sua rede wifi
#define WIFI_PASS "YOUR WIFI PASSWORD"                    //insira entre aspas a sua senha wifi


// ============================================================================
// --- global_variables ---
static EventGroupHandle_t wifi_event_group;
char led1_status[4] = "OFF";                             //string de status do LED1
char led2_status[4] = "OFF";                             //string de status do LED2


// ============================================================================
// --- Functions Prototypes ---
void start_webserver(void);                                        //inicializa o servidor web
esp_err_t root_get_handler(httpd_req_t *req);                      //manipulador de solicitações HTTP GET para o URI raiz
esp_err_t led_get_handler(httpd_req_t *req);                       //manipulador de solicitações HTTP GET, controla LEDs e atualiza a webpage
void wifi_event_handler(void* arg, esp_event_base_t event_base,    //manipulador de eventos Wifi
		                    int32_t event_id, void* event_data);
void wifi_init(void);                                              //inicializa o wifi do ESP32
void select_control(char *buf, char *out, char *param,             //seleciona a saída a ser controlada
		            char *statusL, uint8_t pin);


// ============================================================================
// --- structures initialization ---
static const httpd_uri_t led_control =
{
    .uri       = "/led",                                 //URI associado à estrutura
    .method    = HTTP_GET,                               //método GET (URI responderá a solicitações GET)
    .handler   = led_get_handler,                        //manipulador chamado quando uma solicitação é feita para esta URI
    .user_ctx  = NULL                                    //contexto de usuário. Não utilizado.
};

static const httpd_uri_t root =
{
    .uri       = "/",                                    //apenas uma "/" que associa a URI à raiz do servidor
    .method    = HTTP_GET,                               //método GET (URI responderá a solicitações GET)
    .handler   = root_get_handler,                       //manipulador chamado quando uma solicitação é feita para esta URI
    .user_ctx  = NULL                                    //contexto de usuário. Não utilizado.
};


// ============================================================================
// --- Main Function ---
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();                    //armazena em ret configurações e dados persistentes do NVS

    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)   //verifica erros de inicialização do NVS
    {
        ESP_ERROR_CHECK(nvs_flash_erase());              //apaga o NVS para tentar resolver o problema
        ret = nvs_flash_init();                          //realiza nova tentativa
    }

    ESP_ERROR_CHECK(ret);                                //imprime mensagem de erro caso houver e reinicia o dispositivo

    gpio_pad_select_gpio(LED1);                          //seleciona GPIO para o LED1
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);          //configura GPIO do LED1 como saída
    gpio_pad_select_gpio(LED2);                          //seleciona GPIO para o LED2
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);          //configura GPIO do LED2 como saída
    gpio_pad_select_gpio(LED_STATUS);                    //seleciona GPIO para o LED de STATUS
    gpio_set_direction(LED_STATUS, GPIO_MODE_OUTPUT);    //configura GPIO do LED_STATUS como saída
    gpio_set_level(LED_STATUS, 0);                       //LED de STATUS inicia desligado


    wifi_init();                                                                   //inicializa wifi do ESP32
    xEventGroupWaitBits(wifi_event_group, BIT0, false, true, portMAX_DELAY);       //aguarda indicação de conexão wifi bem sucedida
    start_webserver();                                                             //inicializa o servidor web


} //end app_main


// ============================================================================
// --- Functions ---


// ============================================================================
// --- start_webserver ---
void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();      //estrutura config inicializada com os valores padrão do HTTP
    httpd_handle_t server = NULL;                        //armazena o identificador do servidor web depois que ele for iniciado

    if(httpd_start(&server, &config) == ESP_OK)          //inicializa o servidor HTTP com config. Se resultar em ESP_OK, a inicialização funcinou
    {
        httpd_register_uri_handler(server, &led_control);  //registra um manipulador para o servidor web, que apresenta informações sobre URI, método e HTTP
        httpd_register_uri_handler(server, &root);         //registra um manipulador para o servidor web para lidar com solicitações para a raiz do servidor
    }

} //end start_webserver


// ============================================================================
// --- root_get_handler ---
esp_err_t root_get_handler(httpd_req_t *req)
{

   const char* html_page = generate_html_page(led1_status, led2_status);   //gera a página html de forma dinâmica

   if (html_page != NULL)                                    //verifica se a página foi gerada com sucesso
   {
      httpd_resp_send(req, html_page, strlen(html_page));    //envia a página HTML gerada como resposta à solicitação HTTP
      free((void*)html_page);                                //libera memória alocada para a página
   }

   else                                                      //senão, ocorreu um erro
      httpd_resp_send_500(req);                              //erro interno do servidor (HTTP 500)

    return ESP_OK;                                       //manipulação de solicitação HTTP concluída com sucesso

} //end root_get_handler


// ============================================================================
// --- led_get_handler ---
esp_err_t led_get_handler(httpd_req_t *req)
{
    char*  buf;                                                        //buffer para os parâmetros da URL
    size_t buf_len;                                                    //tamanho do buffer


    buf_len = httpd_req_get_url_query_len(req) + 1;                    //armazena tamanho da string de consulta (query string) da URL

    if(buf_len > 1)                                                    //verifica se a string de consulta não está vazia
    {
       buf = malloc(buf_len);                                          //aloca memória dinamicamente para string de consulta de acordo com buf_len

       if(httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)    //obtém a string de consulta da URL e a armazena em buf
       {
          ESP_LOGI(TAG, "Found URL query => %s", buf);                 //registra a string de consulta no log do sistema
          char param[32];                                              //


          select_control(buf, "led1", param, led1_status, LED1);       //analisa a string de consulta para determinar o estado do LED1
          select_control(buf, "led2", param, led2_status, LED2);       //analisa a string de consulta para determinar o estado do LED2


       } //end if httpd_req_get_url_query_str

        free(buf);                                                     //libera memória alocada para a string de consulta

    } //end if buf_len


    const char* html_page = generate_html_page(led1_status, led2_status);  //gera a página html dinamicamente

    if (html_page != NULL)                                                 //verifica se a página foi gerada com sucesso
    {
        httpd_resp_send(req, html_page, strlen(html_page));                //envia a página HTML gerada como resposta à solicitação HTTP
        free((void*)html_page);                                            //libera memória alocada para a página
    }

    else                                                                   //senão, ocorreu um erro
        httpd_resp_send_500(req);                                          //erro interno do servidor (HTTP 500)

    return ESP_OK;                                                         //manipulação de solicitação HTTP concluída com sucesso

} //end led_get_handler


// ============================================================================
// --- wifi_event_handler ---
void wifi_event_handler(void* arg, esp_event_base_t event_base,
		                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)             //wifi inicializado com sucesso
        esp_wifi_connect();                                                       //inicia a conexão

    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) //erro ao conectar
    {
        esp_wifi_connect();                                                       //realiza nova tentativa de conexão
        xEventGroupClearBits(wifi_event_group, BIT0);                             //limpa bit de evento
        gpio_set_level(LED_STATUS, 0);                                            //desliga LED_STATUS para indicar a perda de conexão
    }

    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)            //obteve um endereço de IP válido?
    {                                                                             //sim
        xEventGroupSetBits(wifi_event_group, BIT0);                               //bit de obtenção de IP é setado
        gpio_set_level(LED_STATUS, 1);                                            //liga LED_STATUS indicando sucesso na conexão

    }


} //end wifi_event_handler


// ============================================================================
// --- wifi_init ---
void wifi_init(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE);                                     //define que os logs de wifi não serão impressos
    wifi_event_group = xEventGroupCreate();                                      //grupo para lidar com eventos da conexão wifi
    esp_netif_init();                                                            //inicializa a camada de interface de rede (netif) do ESP32
    esp_event_loop_create_default();                                             //loop de eventos padrão para lidar com eventos do sistema
    esp_netif_create_default_wifi_sta();                                         //interface de rede padrão para a estação wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();                         //inicialização de estrura de configuração do Wi-Fi com os valores padrão
    esp_wifi_init(&cfg);                                                         //subsistema wifi inicializa com as configurações da estrutura cfg
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);  //registra um manipulador de eventos wifi
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL); //registra um manipulador de eventos de obtenção de IP

    wifi_config_t wifi_config = {                                                //configura as credenciais da rede
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);                                            //modo Wi-Fi para estação (cliente) para que o ESP32 possa se conectar
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);                          //configura as credenciais da rede Wi-Fi para o modo estação
    esp_wifi_start();                                                            //inicia o modo wifi, onde o ESP32 tentará se conectar usando as credenciais

} //end wifi_init


// ============================================================================
// --- select_control ---
void select_control(char *buf, char *out, char *param, char *statusL, uint8_t pin)
{

  if(httpd_query_key_value(buf, out, param, sizeof(param)) == ESP_OK)            //verifica se é possível obter o valor do parâmetro especificado
  {
	ESP_LOGI(TAG, "Found URL query parameter => %s=%s", param, out);             //registra no log do sistema parâmetro encontrado na string de consulta

	if(strcmp(param, "on") == 0)                                                 //se o parâmetro for "on"
	{
	   gpio_set_level(pin, 1);                                                   //liga a saída
	   strcpy(statusL, "ON");                                                    //atualiza o status da página para "ON"
	}

	else if(strcmp(param, "off") == 0)                                           //se o parâmetro for "off"
	{
	   gpio_set_level(pin, 0);                                                   //desliga a saída
	   strcpy(statusL, "OFF");                                                   //atualiza o status da página para "OFF"
	}


  } //end if httpd_query_key_value


} //end select_control







// ============================================================================
// --- End of Program ---

















