#include <SPI.h>

// Prototypes
ISR(SPI0_Handler);

void	SPI_Slave_Initialize (unsigned long Mode);
void	SPI_Mask_Interrupts (void);
void	SPI_Unmask_Interrupts (void);
void	SPI_Print_Data (void);


// Declaration
unsigned long Time_us;
// SPI
#define			NB_DATAS			1000	// Nb data to receive by frame
#define			SPI_TIME_OUT		20	// Time between each frame (us)

typedef struct	StructSpi{
	unsigned int	Counter;
	unsigned long	Last_Time_Rcv;
	unsigned char	Data[NB_DATAS];
	bool			Check_Time_Out;
 bool     Save_Time;
}StruSpi;

StruSpi		Spi0;


// Interruption
ISR (SPI0_Handler)
{
	/*
	// Activate Overrun to check if there is no data lost
	if (REG_SPI0_SR & SPI_SR_OVRES)
	{
		// At least, 1 byte lost 
	}
	*/

	if (REG_SPI0_SR & SPI_SR_RDRF)
	{
    int SR = REG_SPI0_SR;
    
		// Store data received in buffer
		Spi0.Data[Spi0.Counter] = REG_SPI0_RDR;
    //Serial.println(SR, HEX);
    //Serial.println(Spi0.Data[Spi0.Counter]);
		// Active the time out check
    Spi0.Counter ++;
		Spi0.Save_Time = true;
		//Spi0.Last_Time_Rcv = micros();
	}
}

void setup()
{
  // Initialise la liaison série à 9600 bauds
  Serial.begin(9600);

	// SPI initialization
	SPI_Slave_Initialize(SPI_MODE0);
}

void loop()
{
  unsigned char    i;

  Time_us = micros();
  
	if (Spi0.Save_Time == true)
	{
    Spi0.Save_Time = false;
    Spi0.Check_Time_Out = true;
    Spi0.Last_Time_Rcv = Time_us;
	}

  if (Spi0.Check_Time_Out == true)
  {
		if ((Time_us - Spi0.Last_Time_Rcv) > SPI_TIME_OUT)
		{
      SPI_Mask_Interrupts();
      REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
      NVIC_DisableIRQ(SPI0_IRQn);
      NVIC_ClearPendingIRQ(SPI0_IRQn);
      REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
      Spi0.Check_Time_Out = false;
			SPI_Print_Data(Spi0.Counter);
			Spi0.Counter = 0;
      SPI_Slave_Initialize(SPI_MODE0);
    }
	}

  /*while((REG_SPI0_SR & SPI_SR_RDRF) == 0);
  Spi0.Data[Spi0.Counter] = REG_SPI0_RDR;
  Serial.println(Spi0.Data[Spi0.Counter]);*/

/*if ((REG_SPI0_SR & SPI_SR_RDRF) != 0)
  {
    Spi0.Data[0] = REG_SPI0_RDR;
    Serial.println("Début trame");
    for (i = 1; i < NB_DATAS; i ++)
    {
      Spi0.Last_Time_Rcv = micros();
      while((REG_SPI0_SR & SPI_SR_RDRF) == 0);
      Spi0.Data[i] = REG_SPI0_RDR;
      if (micros() - Spi0.Last_Time_Rcv > SPI_TIME_OUT)  { break;  }
    }
    SPI_Print_Data(i);
    Serial.println("Fin trame");
  }
*/
}

void SPI_Slave_Initialize (unsigned long Mode)
{
	
  //Nested Vector Interrupt Controller configuration
  //To set up the handler
  NVIC_DisableIRQ(SPI0_IRQn);
  NVIC_ClearPendingIRQ(SPI0_IRQn);
  NVIC_SetPriority(SPI0_IRQn, 0);
  NVIC_EnableIRQ(SPI0_IRQn);

  // SPI on Arduino DUE
  // MOSI ICSP-4
  // MISO ICSP-1
  // SCK  ICSP-3
  // SS0  pin10
  SPI.begin();
  REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;   // slave and no modefault
  REG_SPI0_CSR = Mode;        // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer
  REG_SPI0_IER = SPI_IER_RDRF;    // active RX interruption on SPI
  //REG_SPI0_IER |= SPI_IER_OVRES;  // active Overrun RX interruption on SPI
  //NVIC_EnableIRQ(SPI0_IRQn);      // active interruptions on SPI
  REG_SPI0_CR = SPI_CR_SPIEN;     // enable SPI
}

void SPI_Mask_Interrupts (void)
{
	REG_SPI0_IMR = SPI_IMR_RDRF;
	//REG_SPI0_IMR |= SPI_IMR_OVRES;
	
}

void SPI_Unmask_Interrupts (void)
{
	REG_SPI0_IMR &= ~(SPI_IMR_RDRF);
	//REG_SPI0_IMR &= ~(SPI_IMR_RDRF | SPI_IMR_OVRES);
}

void SPI_Print_Data (int nb_data)
{
  int   i;

  Serial.println(nb_data);
  for(i = 0; i < nb_data; i++)
  {
    if (i % 12 == 0)
    {
      Serial.print(i);
      Serial.print(":");
      Serial.print(Spi0.Data[i]);
      Serial.print(',');
    }
    if (i % 8 == 0) Serial.println();
  }
  Serial.println();
}

