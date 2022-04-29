
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
* Edits for eece433 lab 2 were done by August Byrne.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
* 04/26/2021 August Byrne
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/******************************************************************************************/
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];
static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;
static q31_t testArray[19] = {-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9};	//fixed-point input array
static q31_t testOutput = 1;	//fixed-point output
static float32_t floatingTestArray[19] = {-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9};	//floating-point input array
static float32_t floatingTestOutput = 2;	//floating-point output
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void  dspTask(void *p_arg);
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};

/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;
    uint32_t size = 19;
    arm_mean_q31(testArray,size,&testOutput);	//fixed-point mean
    for (unsigned int i=0;i<size;i=i+1){		//shifting all of the values in the array
    	testArray[i] = testArray[i] << size;	//in order to avoid overflow issues
    }
    arm_rms_q31(testArray,size,&testOutput);	//fixed-point rms
    testOutput = testOutput >> size;			//output shifting back to get the correct value

    arm_mean_f32(floatingTestArray,size,&floatingTestOutput);	//floating-point mean
    arm_rms_f32(floatingTestArray,size,&floatingTestOutput);	//floating-point rms

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();


}

/*******************************************************************************************
* dspTask
*******************************************************************************************/
static void dspTask(void *p_arg){

    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){

        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        // The following code implements a pass through
        dspOutBuffer[DSP_LEFT_CH][buffer_index] = dspInBuffer[DSP_LEFT_CH][buffer_index]; //Left Channel
        dspOutBuffer[DSP_RIGHT_CH][buffer_index] = dspInBuffer[DSP_RIGHT_CH][buffer_index]; //Right Channel

        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}

/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){

    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];

}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){

    return dspParams.ssize;

}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){

    return dspParams.srate;

}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){

    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];

}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


