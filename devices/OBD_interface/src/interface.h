/* ============================================================================

    interface.h

    ESP32 Web Server

    HTML Page

    Author: Dr. Eng. Wagner Rambo
    Date:   June, 2024

============================================================================ */


const char* generate_html_page(const char* led1_status, const char* led2_status)
{

	// Partes estáticas do HTML
    const char* html_header = "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<title>WR Kits ESP32 Server</title>"
        "<style>"
        "html{font-family: Tahoma;margin: 0px auto;text-align: center;background-color: #FEFFFF;}"
        ".bts{border: none;color: #FFFFFF;padding: 14px 36px;text-decoration: none;font-size: 23px;margin: 1.5px;cursor: pointer;font-family: Verdana;}"
        ".btOn{background-color: #33A590;}"
        ".btOff{background-color: #777777;}"
        "</style>"
        "</head>"
        "<body>"
        "<h2>WR Kits ESP32 Web Server</h2>";

    const char* html_footer = "</body></html>";

    // Estimativa do tamanho necessário para a string HTML
    size_t html_size = strlen(html_header) + strlen(html_footer) +
                       strlen("<p>LED 1 Status: </p>"
                    		  "<button class=\"bts btOn\"  onclick=\"location.href='/led?led1=on'\">Turn LED On</button>"
                    		  "<button class=\"bts btOff\" onclick=\"location.href='/led?led1=off'\">Turn LED Off</button>") + strlen(led1_status) +
                       strlen("<p>LED 2 Status: </p>"
                    		   "<button class=\"bts btOn\"  onclick=\"location.href='/led?led2=on'\">Turn LED On</button>"
                    		   "<button class=\"bts btOff\" onclick=\"location.href='/led?led2=off'\">Turn LED Off</button>") + strlen(led2_status) + 1;

    char* html_page = (char*)malloc(html_size);    //aloca memória dinâmica

    if(html_page == NULL)
       return NULL;                                //falha na alocação de memória


    // Preenchendo a string HTML com o conteúdo dinâmico
    snprintf(html_page, html_size,
        "%s"
        "<p>LED 1 Status: %s</p>"
        "<button class=\"bts btOn\"  onclick=\"location.href='/led?led1=on'\">Turn LED On</button>"
        "<button class=\"bts btOff\" onclick=\"location.href='/led?led1=off'\">Turn LED Off</button>"
        "<p>LED 2 Status: %s</p>"
        "<button class=\"bts btOn\"  onclick=\"location.href='/led?led2=on'\">Turn LED On</button>"
        "<button class=\"bts btOff\" onclick=\"location.href='/led?led2=off'\">Turn LED Off</button>"
        "%s",
        html_header, led1_status, led2_status, html_footer);

    return html_page;

} //end generate_html_page




// ============================================================================
// --- End interface.h ---






