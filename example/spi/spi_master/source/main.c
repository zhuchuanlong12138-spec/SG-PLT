/******************************************************************************
* Copyright (C) 2016, Xiaohua Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Xiaohua Semiconductor Co.,Ltd ("XHSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with XHSC
* components. This software is licensed by XHSC to be adapted only
* for use in systems utilizing XHSC components. XHSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. XHSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* XHSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* XHSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2016-02-16  1.0  Devi First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "spi.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define     T1_PORT                 (3)
#define     T1_PIN                  (2)
/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
__IO uint8_t tx_cnt, rx_cnt;
volatile uint8_t tx_buf[1]={0x9f}, rx_buf[10], tmp_buf[3]={0x1,0x2,0x15};
/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/

int32_t main(void)
{
    stc_spi_config_t  SPIConfig;
    volatile uint8_t tmp = 0;
    tx_cnt = 0;

    
    Clk_SwitchTo(ClkXTH);
    Clk_SetPeripheralGate(ClkPeripheralSpi,TRUE);    
    
    Gpio_InitIO(T1_PORT, T1_PIN, GpioDirOut);
    Gpio_SetIO(T1_PORT, T1_PIN, 0);
    
    Gpio_SetFunc_SPI_CS_P03();
    Gpio_SetFunc_SPIMISO_P23();
    Gpio_SetFunc_SPIMOSI_P24();
    Gpio_SetFunc_SPICLK_P25();
    
    Spi_SetCS(TRUE);
    //配置SPI
    SPIConfig.bCPHA = Spicphafirst;
    SPIConfig.bCPOL = Spicpollow;
    SPIConfig.bIrqEn = FALSE;
    SPIConfig.bMasterMode = SpiMaster;
    SPIConfig.u8BaudRate = SpiClkDiv32;
    SPIConfig.pfnIrqCb = NULL;

    Spi_Init(&SPIConfig);
    Gpio_InitIO(T1_PORT, T1_PIN, GpioDirOut);
    Gpio_SetIO(T1_PORT, T1_PIN, 0);   


    ///< 片选，开始通讯
    Spi_SetCS(FALSE);

    ///< 主机向从机发送数据
    while(tx_cnt<1)                                        //主机发送数据给从机
    {
        Spi_SendData(tx_buf[tx_cnt++]);
        //while(Spi_GetStatus( SpiIf) == FALSE);    //等待发送结束
    }
    Spi_SetCS(TRUE);
    delay1ms( 1000);
    Spi_SetCS(FALSE);
    ///< 主机接收从机数据
    while(rx_cnt<3)                                        //接收从机的数据
    {
        rx_buf[rx_cnt++] = Spi_ReceiveData();
    }

    ///< 结束通信
    Spi_SetCS(TRUE);

    ///< 判断发送的数据与接收的数据是否相等
    for(tmp = 0; tmp<3; )
    {
        if(rx_buf[tmp] == tmp_buf[tmp])
            tmp++;
        else
            break;
    }
    if(tmp == 3)                                    //如果接收到的数据与发送的数据相等则置高
        Gpio_SetIO(T1_PORT, T1_PIN, 1);
    	

    while(1);


}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


