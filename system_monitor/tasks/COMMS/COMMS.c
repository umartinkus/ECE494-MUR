#include <stdio.h>
#include "COMMS.h"
#include "spi.h"
#include "esp_err.h"

void COMMS(void *args)
{
    while(spi3_slave_init() != ESP_OK){

    }
    for(;;){
        
    }
}
