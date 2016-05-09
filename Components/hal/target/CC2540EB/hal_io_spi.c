#include"hal_defs.h"
#include"hal_types.h"
#include"iocc2541.h"
#include"hal_io_spi.h"



void hal_spi_init()
{

P1DIR|=SPI_MOSI_BIT_POS|SPI_CLK_BIT_POS|SPI_CS_BIT_POS;

SPI_CLK_DOWN();
}

static void hal_spi_trans_byte(uint8 trans_data)
{
  for(uint8 i=0;i<8;i++)
  {
  if(trans_data&0x80)
  SPI_MOSI_DATA_SET();
  else
  SPI_MOSI_DATA_CLR();
  
  SPI_CLK_UP();
  SPI_CLK_DOWN();
  trans_data<<=1;
  }
}

static uint8 hal_spi_recv_byte()
{
  uint8 recv_data=0;
  uint8 i=7;
  do
  {
  SPI_CLK_UP(); 
  if(SPI_MISO_DATA_READ())
  recv_data|=1<<i;
  SPI_CLK_DOWN();
  }
  while(i--!=0);
 
  return recv_data;
}

void hal_spi_trans(uint8 *pbuff,uint16 len)
{
  while(len-->0)
  {
    hal_spi_trans_byte(*pbuff++);
  }

}

void hal_spi_recv(uint8 *pbuff,uint16 len)
{
  while(len-->0)
  {
  *pbuff++=hal_spi_recv_byte();
  }
  
}

