#include "stdio.h"
#include "AD5940.h"
#include "AD5940Function.h"
#include "Impedance.h"
#include "main.h"
void print_rststa(uint32_t reg);
void AD5940_Reset(void);
void AD5940_messege(void);
static void AD5940_PGA_Calibration(void);
void AD5940_ADC(void);
void AD5940ImpedanceStructInit(void);
void AD5940_impedance_init(void);
void AD5940_impedance(void);
static int32_t AD5940_impedance_sys(void);
int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

/*       ADC    有问题  */
#define ADCPGA_GAIN_SEL   ADCPGA_1P5
static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

void AD5940_ADC(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  AD5940_PGA_Calibration();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
  adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  //AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_VSET1P1);   /* Optionally, you can change ADC MUX with this function */

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_ADCPowerCtrlS(bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);

  while(1)
  {
    uint32_t rd;
   if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_SINC2RDY))  
    {
     static uint32_t count;
      AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
      rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      count ++;
      /* ADC Sample rate is 800kSPS. SINC3 OSR is 4, SINC2 OSR is 1333. So the final output data rate is 800kSPS/4/1333 = 150.0375Hz */
      if(count%150==0) /* Print data @1Hz */
      {
       // count = 0;
        float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
        printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f \r\n",rd, diff_volt, diff_volt+1.11);
//        if (count==1500){
//         AD5940_Reset();
//        count=0;
//        }
      }
    }
  }
}



/*       AD5940_messege      */
void AD5940_messege(void)
{
  unsigned long temp;
  AD5940_HWReset();
  AD5940_Initialize();
  temp = AD5940_ReadReg(REG_AFECON_ADIID);
  printf("Read ADIID register, got: 0x%04lx\n", temp);
  if(temp != AD5940_ADIID)
    printf("Read register test failed.\n" );
  else
    printf("Read register test pass\n");
}






/*       reset      */
void AD5940_Reset(void)
{
  uint32_t temp;
  printf("Wait 5 secondes\n");
  AD5940_Delay10us(100*5000); /* Delay 5s */
  printf("\n1. AD5940 Power ON\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);  /* Clear reset status. This register will remain its value until we manually clear it. Reset operation won't reset this register. */

  printf("\n2. Perform Hardware reset now!\n");
  AD5940_HWReset();
  printf("Hardware reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);

  printf("\n3. Perform Software Reset now \n");
  AD5940_SoftRst();
  printf("Software reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  printf("\nReset Test done \n");
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
  */
  AD5940_Initialize();
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf); /* Clear reset status register. */

  while(1)
  {
    printf("reset while\r\n");
  HAL_Delay(1000);
  }
    
}


void print_rststa(uint32_t reg)
{
  printf("<<<<<<<Reset Status<<<<<\n");
  if(reg & 0x01)
    printf("POR Reset Happened\n");
  if(reg & 0x02)
    printf("Hardware/External Reset Happened\n");
  if(reg & 0x08)
    printf("Software Reset Happened\n");
  if((reg&0xb) == 0)
    printf("No reset happened\n");
  printf(">>>>>>>Reset Status Done>>>>>\n");
}



/*       阻抗impedance      */




static int32_t AD5940_impedance_sys(void)
{
CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}  



void AD5940_impedance(void)
{
  uint32_t temp;  
 
  while(1)
  {
   if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();        //清楚标志位，进中断处理
      temp = APPBUFF_SIZE;
     AppIMPISR(AppBuff, &temp);     //有数据就进中断，处理数据
        ImpedanceShowResult(AppBuff, temp);//展示数据
    }
  }
}


int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f  datacount:%d \r\n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI,DataCount);
  }
  return 0;
}


void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 10000.0;
  pImpedanceCfg->SinFreq = 60000.0;
  pImpedanceCfg->FifoThresh = 4;
	
	/* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
	/* Note the RCAL0 resistor is 10kOhm. */
	pImpedanceCfg->DswitchSel = SWD_CE0;
	pImpedanceCfg->PswitchSel = SWP_RE0;
	pImpedanceCfg->NswitchSel = SWN_SE0;
	pImpedanceCfg->TswitchSel = SWT_SE0LOAD;
	/* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
	pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_5K;	
	
	/* Configure the sweep function. */
	pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
	pImpedanceCfg->SweepCfg.SweepStart = 100.0f;	/* Start from 1kHz */
	pImpedanceCfg->SweepCfg.SweepStop = 100e3f;		/* Stop at 100kHz */
	pImpedanceCfg->SweepCfg.SweepPoints = 101;		/* Points is 101 */
	pImpedanceCfg->SweepCfg.SweepLog = bTRUE;
	/* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
	pImpedanceCfg->PwrMod = AFEPWR_HP;
	/* Configure filters if necessary */
	pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_2;		/* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;
}


void AD5940_impedance_init(void)
{
AD5940_impedance_sys();
AD5940ImpedanceStructInit();
AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */
}




