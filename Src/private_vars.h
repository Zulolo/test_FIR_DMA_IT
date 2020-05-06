#ifndef PRIVATE_VARS
#define PRIVATE_VARS

/**********************--======  FMAC  ======--***********************/

/* Array of filter coefficients B (feed-forward taps) in Q1.15 format */
static int16_t aFilterCoeffB[COEFF_VECTOR_B_SIZE] =
{
     2212,  8848, 13272,  8848,  2212
};

/* Array of input values in Q1.15 format (in four parts in order to write new data during the calculation) */
static int16_t aInputDummyValues[INPUT_DUMMY_ARRAY_SIZE] = {0, 0, 0, 0};

/* Expected number of calculated samples for the used aCalculatedFilteredDataX */
uint16_t CurrentInputArraySize;

/* Expected number of calculated samples for the used aCalculatedFilteredDataX */
uint16_t ExpectedCalculatedFilteredDataSize;

/* Status of the calculation */
__IO uint32_t FilterConfigCallbackCount    = 0;
__IO uint32_t FilterPreloadCallbackCount   = 0;
__IO uint32_t HalfGetDataCallbackCount     = 0;
__IO uint32_t GetDataCallbackCount         = 0;
__IO uint32_t OutputDataReadyCallbackCount = 0;
__IO uint32_t ErrorCount                   = 0;


/* Auxiliary counter */
uint32_t Index;

/**********************--======  ADCs  ======--***********************/

__IO int16_t uhADCxConvertedData[ADC_BUF_LENGTH];        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO int16_t uhFilteredData[ADC_BUF_LENGTH];   
#endif
