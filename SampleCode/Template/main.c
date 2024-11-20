/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "Mini51Series.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_ENABLE_SCHMITT                  		(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_DISABLE_SCHMITT                 		(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_ERASE_KEY                             (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


#define DATA_FLASH_TEST_BASE                            (0x3E00)    // 15.5K
unsigned int ram_buffer[FMC_FLASH_PAGE_SIZE/4] = {0};

#define KEY_ADDR                                        (0x00)
#define KEY_0                                           (0xFFFF)    // ERASE
#define KEY_1                                           (0x5A5A)    // enable
#define KEY_2                                           (0x7F7F)    // disable

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }

void LOCK_FLASH(void)
{
    FMC_DISABLE_AP_UPDATE();

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
}

void UNLOCK_FLASH(void)
{
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    FMC_ENABLE_AP_UPDATE();    
}


void write_data_flash(unsigned int Byte4_alignment,unsigned int WriteData)
{
    uint32_t u32Addr = 0;
    // uint32_t u32data = 0;
    uint32_t i = 0;

    if (Byte4_alignment >= (FMC_FLASH_PAGE_SIZE/4) )
    {        
        printf("over address size(0x%04X)\r\n" , Byte4_alignment);
        return;
    }

    UNLOCK_FLASH();

    // read back data to ram
    for (u32Addr = DATA_FLASH_TEST_BASE , i = 0 ; i < (FMC_FLASH_PAGE_SIZE/4) ; u32Addr += 4 , i += 4)
    {
        ram_buffer[i] = FMC_Read(u32Addr);
        // printf("u32Addr:0x%08X,i:0x%08X\r\n",u32Addr,i);
    }

    #if 1   // debug ram bufer
    printf("\r\nBEFORE WRITE\r\n");
    for (i = 0 ; i < (FMC_FLASH_PAGE_SIZE/4) ; i += 4 )
    {
        printf("0x%08X,",ram_buffer[i]);

        if ((i+4)%4 ==0)
        {
            printf("\r\n");
        }            
    }
    #endif

    //udpate data in ram
    ram_buffer[Byte4_alignment] = WriteData;

    //erase page
    FMC_Erase(DATA_FLASH_TEST_BASE);

    // write ram data into dataflash
    for (u32Addr = DATA_FLASH_TEST_BASE , i = 0 ; i < (FMC_FLASH_PAGE_SIZE/4) ; u32Addr += 4 , i += 4)
    {        
        FMC_Write(u32Addr, ram_buffer[i]);
    }

    // read back to verify
    #if 1
    // read back data to ram
    for (u32Addr = DATA_FLASH_TEST_BASE , i = 0 ; i < (FMC_FLASH_PAGE_SIZE/4) ; u32Addr += 4 , i += 4)
    {
        ram_buffer[i] = FMC_Read(u32Addr);
    }

    #if 1   // debug ram bufer
    printf("\r\nAFTER WRITE\r\n");
    for (i = 0 ; i < (FMC_FLASH_PAGE_SIZE/4) ; i += 4 )
    {
        printf("0x%08X,",ram_buffer[i]);

        if ((i+4)%4 ==0)
        {
            printf("\r\n");
        }            
    }
    #endif
    #endif
    
    LOCK_FLASH();
}

void read_data_flash(unsigned int Byte4_alignment,unsigned int* ReadData)
{
    uint32_t u32Addr = 0;

    if (Byte4_alignment >= (FMC_FLASH_PAGE_SIZE/4) )
    {        
        printf("over address size(0x%04X)\r\n" , Byte4_alignment);
        return;
    }


    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    u32Addr = Byte4_alignment + DATA_FLASH_TEST_BASE; 

    *ReadData = FMC_Read(u32Addr);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
}

int set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
    {        
        printf("data flash already set\r\n\n");

        return 0;
    }

    FMC_ENABLE_CFG_UPDATE();

    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;

    printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_TEST_BASE);

    // Perform chip reset to make new User Config take effect
    SYS->IPRSTC1 = SYS_IPRSTC1_CHIP_RST_Msk;
    return 0;
}

void Data_Flash_Init(void)
{
    uint32_t u32Data = 0;

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    set_data_flash_base(DATA_FLASH_TEST_BASE);

    /* Read Data Flash base address */
    u32Data = FMC_ReadDataFlashBaseAddr();
    printf("Data Flash Base Address ............... [0x%08X]\n", u32Data);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

}

void enable_Schmitt_process(void)
{   
    uint32_t u32data = 0;

    if (FLAG_PROJ_ENABLE_SCHMITT)
    {
        FLAG_PROJ_ENABLE_SCHMITT = 0;
        printf("enable Schmitt\r\n");

        read_data_flash(KEY_ADDR , &u32data);
        if (u32data == KEY_1)
        {
            printf("ALREADY enable Schmitt\r\n");        
        }
        else
        {
            printf("key data:0x%08X\r\n",u32data);  
            write_data_flash(KEY_ADDR,KEY_1);
            
            while(!UART_IS_TX_EMPTY(UART));
            SYS_UnlockReg();
            // NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
            // SYS_ResetCPU();     // Not reset I/O and peripherals
            SYS_ResetChip();  
        }
        
    }

    if (FLAG_PROJ_DISABLE_SCHMITT)
    {
        FLAG_PROJ_DISABLE_SCHMITT = 0;
        printf("disable Schmitt\r\n");

        read_data_flash(KEY_ADDR , &u32data);
        if (u32data == KEY_2)
        {
            printf("ALREADY disable Schmitt\r\n");        
        }
        else
        {
            printf("key data:0x%08X\r\n",u32data);  
            write_data_flash(KEY_ADDR,KEY_2);
        
            while(!UART_IS_TX_EMPTY(UART));
            SYS_UnlockReg();
            // NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
            // SYS_ResetCPU();     // Not reset I/O and peripherals
            SYS_ResetChip();  
        }
    }

    if (FLAG_PROJ_ERASE_KEY)
    {
        FLAG_PROJ_ERASE_KEY = 0; 
        printf("ERASE KEY DATA\r\n");  
        write_data_flash(KEY_ADDR,KEY_0);
        
        while(!UART_IS_TX_EMPTY(UART));
        SYS_UnlockReg();
        // NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
        // SYS_ResetCPU();     // Not reset I/O and peripherals
        SYS_ResetChip();  
    }
}

void EINT1_IRQHandler(void)
{
    /* For P5.2, clear the INT flag */
    P5->ISRC = BIT2;
    P24 ^= 1;
    printf("P5.2 EINT1 occurred. \n");
}


void EINT1_Init(void)
{
    uint32_t u32data = 0;
    
    GPIO_SetMode(P2, BIT4, GPIO_PMD_OUTPUT); 
    
    GPIO_SetMode(P5, BIT2, GPIO_PMD_INPUT);  

    
    read_data_flash(KEY_ADDR , &u32data);
    switch(u32data)
    {   
        case KEY_1://enable schmitt 
            printf("read key:0x%08X(ENABLE SCHMITT)\r\n",u32data);
            SYS->P5_MFP = (SYS_MFP_P52_INT1) | SYS_MFP_TYPE_Msk(2);
            break;
        case KEY_2://disable schmitt
            printf("read key:0x%08X(DISABLE SCHMITT)\r\n",u32data);
            SYS->P5_MFP = (SYS_MFP_P52_INT1);
            break;

        default:
            printf("NOT set schmitt key:0x%08X\r\n",u32data);
            SYS->P5_MFP = (SYS_MFP_P52_INT1);
            break;
    }

    GPIO_ENABLE_DEBOUNCE(P5, BIT2);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_2048);

    GPIO_EnableEINT1(P5, 2, GPIO_INT_RISING);
    NVIC_SetPriority(EINT1_IRQn, 1);
    NVIC_EnableIRQ(EINT1_IRQn);

}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        P36 ^= 1;        
    }

    enable_Schmitt_process();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART);


	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
                FLAG_PROJ_ENABLE_SCHMITT = 1;
				break;
			case '2':
                FLAG_PROJ_DISABLE_SCHMITT = 1;
				break;
			case '3':
                FLAG_PROJ_ERASE_KEY = 1;
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART, UART_IER_RDA_IEN_Msk | UART_IER_RTO_IEN_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART->FSR & (UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk))
    {
        UART_ClearIntFlag(UART, (UART_ISR_RLS_INT_Msk| UART_ISR_BUF_ERR_INT_Msk));
    }	
}

void UART_Init(void)
{
    SYS_ResetModule(UART_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART, 115200);
    UART_EnableInt(UART, UART_IER_RDA_IEN_Msk | UART_IER_RTO_IEN_Msk);
    NVIC_EnableIRQ(UART_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->P3_MFP = (SYS->P3_MFP & ~(SYS_MFP_P36_Msk)) | (SYS_MFP_P36_GPIO);

    GPIO_SetMode(P3, BIT6, GPIO_PMD_OUTPUT);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

//    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
//    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

//    /* Enable external 12MHz XTAL (UART), and internal 22.1184MHz */
//    CLK->PWRCON = CLK_PWRCON_XTL12M ;

//    /* Waiting for clock ready */
//    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk );

     /* Enable external 12MHz XTAL (UART), and internal 22.1184MHz */
    CLK->PWRCON =  CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk);   

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_IRC22M,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_HCLK_S_IRC22M);

    CLK_EnableModuleClock(UART_MODULE);
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_IRC22M,CLK_CLKDIV_UART(1));

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE,CLK_CLKSEL1_TMR1_S_IRC22M,0);


    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

	GPIO_Init();
	UART_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    EINT1_Init();
    Data_Flash_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
