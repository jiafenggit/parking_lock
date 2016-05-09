#ifndef __HAL_IO_SPI_H__
#define __HAL_IO_SPI_H__


#define  BIT_POS(x)        (1<<x)

#define  SPI_PORT          P1

#define  SPI_MOSI_PORT_IDEX  5
#define  SPI_MISO_PORT_IDEX  4
#define  SPI_CLK_PORT_IDEX   3
#define  SPI_CS_PORT_IDEX    2

#define  SPI_MOSI_BIT_POS    BIT_POS(SPI_MOSI_PORT_IDEX)
#define  SPI_MISO_BIT_POS    BIT_POS(SPI_MISO_PORT_IDEX)
#define  SPI_CLK_BIT_POS     BIT_POS(SPI_CLK_PORT_IDEX)
#define  SPI_CS_BIT_POS      BIT_POS(SPI_CS_PORT_IDEX)


#define  SPI_CLK_DOWN()       SPI_PORT&=~SPI_CLK_BIT_POS
#define  SPI_CLK_UP()         SPI_PORT|= SPI_CLK_BIT_POS

#define  SPI_CS_SELECT()      SPI_PORT&=~SPI_CS_BIT_POS 
#define  SPI_CS_DESELECT()    SPI_PORT|= SPI_CS_BIT_POS

#define  SPI_MOSI_DATA_SET()   SPI_PORT|=SPI_MOSI_BIT_POS
#define  SPI_MOSI_DATA_CLR()   SPI_PORT&=~SPI_MOSI_BIT_POS


#define  SPI_MISO_DATA_READ()      SPI_PORT&SPI_MISO_BIT_POS



void hal_spi_init();
void hal_spi_trans(uint8 *pbuff,uint16 len);
void hal_spi_recv(uint8 *pbuff,uint16 len);













#endif