#include <stdint.h>

#define MOSI PB3
#define MISO PB4
#define SCK PB5
#define CS PB2

void spiInit();
char SPI_Recv(void);