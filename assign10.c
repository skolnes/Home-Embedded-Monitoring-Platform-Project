//
// GENG 422 - Home Embedded Monitoring Platform
//
// Scott Kolnes
//

#include <stdio.h>

#include "u8g_arm.h"
#include "wiced.h"

#define FALSE                   0
#define TRUE                    1

#define FONT_HEIGHT             10
#define FONT_WIDTH              6

#define MB_DEBOUNCE_COUNT       2

#define OLED_I2C_ADDR           0x3C

#define PSOC_I2C_ADDR           0x42
#define PSOC_BTN_REG_ADDR       0x06
#define PSOC_TEMP_REG_ADDR      0x07
#define PSOC_HUM_REG_ADDR       0x0B
#define PSOC_LIGHT_REG_ADDR     0x0F
#define PSOC_POT_REG_ADDR       0x13

#define PSOC_BTN_REG_PROX       0x40
#define PSOC_BTN_REG_MB2        0x20
#define PSOC_BTN_REG_MB1        0x10
#define PSOC_BTN_REG_CSB3       0x08
#define PSOC_BTN_REG_CSB2       0x04
#define PSOC_BTN_REG_CSB1       0x02
#define PSOC_BTN_REG_CSB0       0x01

#define PSOC_POT_OFFSET         0.0
#define PSOC_POT_SCALE          45.0

#define PSOC_TEMP_OFFSET        1.8
#define PSOC_TEMP_SCALE        32.0

#define PSOC_LIGHT_OFFSET        0.25

#define SYS_MGR_PRIORITY        10
#define SYS_MGR_STACK_SIZE      1024

#define UI_IN_PRIORITY          10
#define UI_IN_STACK_SIZE        1024

#define UI_IN_SAMPLE_PERIOD     25

#define UI_OUT_PRIORITY         8
#define UI_OUT_STACK_SIZE       1024

#define UI_OUT_STRBUF_SIZE      5

#define USER_NAME_CHARS         2
#define USER_NUM                12

#define TEMP_AVG_SAMPLES        10
#define HUM_AVG_SAMPLES         10
#define LIGHT_AVG_SAMPLES       15

#define PASSWORD_LENGTH         4


enum HighlightStateType {HL_NONE, HL_TEMP_LOW, HL_TEMP_HIGH, HL_HUM_LOW,
                         HL_HUM_HIGH, HL_LIGHT_LOW, HL_LIGHT_HIGH, HL_WRAP};

enum MBStateType {MB_RELEASED, MB_PRESSED, MB_DEBOUNCE};

enum PasswordStateType {PASSWORD_STATE_1, PASSWORD_STATE_2, PASSWORD_STATE_3, PASSWORD_STATE_4};

enum CapSenseStateType {CAP_1, CAP_2, CAP_3, CAP_4};


typedef int Bool;

typedef struct _UserType
{
    char *  NamePtr;
    Bool    IsConnected;
} UserType;

typedef struct _UiInDataType
{
    Bool        MB1IsPressed;
    Bool        MB2IsPressed;
    Bool        CSB0IsPressed;
    Bool        CSB1IsPressed;
    Bool        CSB2IsPressed;
    Bool        CSB3IsPressed;
    Bool        ProxDetected;
    int         Temp;
    int         Hum;
    int         Light;
    int         Pot;
} UiInDataType;

typedef struct _UiOutDataType
{
    Bool        Armed;
    int         NewVal;
    int         HighlightState;
    int         TempCurVal;
    int         TempLowLimit;
    int         TempHighLimit;
    Bool        TempLowAlarm;
    Bool        TempHighAlarm;
    int         HumCurVal;
    int         HumLowLimit;
    int         HumHighLimit;
    Bool        HumLowAlarm;
    Bool        HumHighAlarm;
    int         LightCurVal;
    int         LightLowLimit;
    int         LightHighLimit;
    Bool        LightLowAlarm;
    Bool        LightHighAlarm;
    UserType    User[USER_NUM];
} UiOutDataType;

typedef struct _WinAvgIntType
{
    int         NumSamples;
    int *       BufPtr;
    Bool        BufIsFull;
    int         Div;
    int         Idx;
    int         Sum;
} WinAvgIntType;


u8g_t               u8gState;
UiInDataType        UiInData = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
                                0.0, 0.0, 0.0, 0.0};
wiced_semaphore_t   UiInSemaphoreHandle;
UiOutDataType       UiOutData;
wiced_semaphore_t   UiOutSemaphoreHandle;


void SystemManagerThread(wiced_thread_arg_t Arg);
void UiInThread(wiced_thread_arg_t Arg);
void UiOutThread(wiced_thread_arg_t Arg);
void DrawString(int XPos, int YPos, char * StrPtr, Bool Invert);
void ToggleLed1(void);
int WinAvgInt(int NewVal, WinAvgIntType * WinAvgDataPtr);


void application_start()
{
    wiced_thread_t  SystemManagerThreadHandle;
    wiced_thread_t  UiInThreadHandle;
    wiced_thread_t  UiOutThreadHandle;

    wiced_init();

    wiced_rtos_init_semaphore(&UiInSemaphoreHandle);

    wiced_rtos_init_semaphore(&UiOutSemaphoreHandle);

    wiced_rtos_create_thread(&UiOutThreadHandle, UI_OUT_PRIORITY, "UiOutThread",
                             UiOutThread, UI_OUT_STACK_SIZE, NULL);

    wiced_rtos_create_thread(&SystemManagerThreadHandle, SYS_MGR_PRIORITY,
                             "SystemManagerThread", SystemManagerThread,
                             SYS_MGR_STACK_SIZE, NULL);

    wiced_rtos_create_thread(&UiInThreadHandle, UI_IN_PRIORITY, "UiInThread",
                             UiInThread, UI_IN_STACK_SIZE, NULL);
}


void SystemManagerThread(wiced_thread_arg_t Arg)
{
    Bool SelectOneShot = TRUE;
    Bool EnterOneShot = TRUE;

    Bool Cap1Pressed = FALSE;
    Bool Cap2Pressed = FALSE;
    Bool Cap3Pressed = FALSE;
    Bool Cap4Pressed = FALSE;

    int PasswordState = PASSWORD_STATE_1;
    Bool Password_Arr[PASSWORD_LENGTH];

    while (TRUE)
    {
        // Wait for new UI input data
        wiced_rtos_get_semaphore(&UiInSemaphoreHandle, WICED_WAIT_FOREVER);

        UiOutData.NewVal = UiInData.Pot;
        UiOutData.TempCurVal = UiInData.Temp;
        UiOutData.HumCurVal = UiInData.Hum;
        UiOutData.LightCurVal = UiInData.Light;

        Password_Arr[0] = Cap2Pressed;
        Password_Arr[1] = Cap3Pressed;
        Password_Arr[2] = Cap1Pressed;
        Password_Arr[3] = Cap4Pressed;

        // Check for select button pressed
        if (UiInData.MB2IsPressed && SelectOneShot)
        {
            SelectOneShot = FALSE;
            UiOutData.HighlightState++;
            if (UiOutData.HighlightState == HL_WRAP)
            {
                UiOutData.HighlightState = HL_NONE;
            }
        }
        if (!UiInData.MB2IsPressed)
        {
            SelectOneShot = TRUE;
        }

        // Check for enter button pressed
        if (UiInData.MB1IsPressed && EnterOneShot)
        {
            EnterOneShot = FALSE;
            switch(UiOutData.HighlightState)
            {
            case HL_NONE:
                // do nothing
                break;

            case HL_TEMP_LOW:
                UiOutData.TempLowLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;

            case HL_TEMP_HIGH:
                UiOutData.TempHighLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;

            case HL_HUM_LOW:
                UiOutData.HumLowLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;

            case HL_HUM_HIGH:
                UiOutData.HumHighLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;

            case HL_LIGHT_LOW:
                UiOutData.LightLowLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;

            case HL_LIGHT_HIGH:
                UiOutData.LightHighLimit = UiOutData.NewVal;
                UiOutData.HighlightState = HL_NONE;
                break;
            }

            // Check the temperature, humidity, and light limits
            UiOutData.TempLowAlarm = (UiOutData.TempCurVal < UiOutData.TempLowLimit);
            UiOutData.TempHighAlarm = (UiOutData.TempCurVal > UiOutData.TempHighLimit);
            UiOutData.HumLowAlarm = (UiOutData.HumCurVal < UiOutData.HumLowLimit);
            UiOutData.HumHighAlarm = (UiOutData.HumCurVal > UiOutData.HumHighLimit);
            UiOutData.LightLowAlarm = (UiOutData.LightCurVal < UiOutData.LightLowLimit);
            UiOutData.LightHighAlarm = (UiOutData.LightCurVal > UiOutData.LightHighLimit);
        }
        if (!UiInData.MB1IsPressed)
        {
            EnterOneShot = TRUE;
        }

        if (UiInData.CSB0IsPressed)
        {
            Cap1Pressed = TRUE;
        }

        if(UiInData.CSB1IsPressed)
        {
            Cap2Pressed = TRUE;
        }

        if(UiInData.CSB2IsPressed)
        {
            Cap3Pressed = TRUE;
        }

        if(UiInData.CSB3IsPressed)
        {
            Cap4Pressed = TRUE;
        }

        switch(PasswordState)
        {
        case PASSWORD_STATE_1:
            if(UiInData.CSB0IsPressed | UiInData.CSB1IsPressed | UiInData.CSB2IsPressed | UiInData.CSB3IsPressed)
            {
                if(Password_Arr[0] == TRUE)
                {
                    PasswordState = PASSWORD_STATE_2;
                }
                else
                {
                    PasswordState = PASSWORD_STATE_1;
                }
            }
            break;
        case PASSWORD_STATE_2:
            if(UiInData.CSB0IsPressed | UiInData.CSB1IsPressed | UiInData.CSB2IsPressed | UiInData.CSB3IsPressed)
            {
                if(Password_Arr[1] == TRUE)
                {
                     PasswordState = PASSWORD_STATE_3;
                }
                else
                {
                     PasswordState = PASSWORD_STATE_1;
                     Cap2Pressed = FALSE;
                }
            }
            break;
        case PASSWORD_STATE_3:
            if(UiInData.CSB0IsPressed | UiInData.CSB1IsPressed | UiInData.CSB2IsPressed | UiInData.CSB3IsPressed)
            {
                if(Password_Arr[2] == TRUE)
                {
                    PasswordState = PASSWORD_STATE_4;
                }
                else
                {
                    PasswordState = PASSWORD_STATE_1;
                    Cap2Pressed = FALSE;
                    Cap3Pressed = FALSE;
                }
            }
            break;
        case PASSWORD_STATE_4:
            if(UiInData.CSB0IsPressed | UiInData.CSB1IsPressed | UiInData.CSB2IsPressed | UiInData.CSB3IsPressed)
            {
                if(Password_Arr[3] == TRUE)
                {
                    PasswordState = PASSWORD_STATE_1;
                    UiOutData.Armed = UiOutData.Armed ? FALSE:TRUE;

                    Cap2Pressed = FALSE;
                    Cap3Pressed = FALSE;
                    Cap1Pressed = FALSE;
                    Cap4Pressed = FALSE;
                }
                else
                {
                    PasswordState = PASSWORD_STATE_1;
                    Cap2Pressed = FALSE;
                    Cap3Pressed = FALSE;
                    Cap1Pressed = FALSE;
                }
            }
            break;
        }

        // Notify the UI output thread to update the display
        wiced_rtos_set_semaphore(&UiOutSemaphoreHandle);
    }
}


void UiInThread(wiced_thread_arg_t Arg)
{
    typedef struct _PsocRegs
    {
        uint8_t Btns;
        float   Temp;
        float   Hum;
        float   Light;
        float   Pot;
    } PsocRegsType;


    int             MB1DebounceCount;
    int             MB1State = MB_RELEASED;
    int             MB2DebounceCount;
    int             MB2State = MB_RELEASED;
    PsocRegsType    PsocRegs;
    uint8_t         PsocRegAddrBuf;
    UiInDataType    UiInDataNew;

    wiced_i2c_device_t I2cPsoc =
    {
        .port          = WICED_I2C_2,
        .address       = PSOC_I2C_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .flags         = 0,
        .speed_mode    = I2C_HIGH_SPEED_MODE,
    };

    UiInDataNew.MB1IsPressed  = FALSE;
    UiInDataNew.MB2IsPressed  = FALSE;
    UiInDataNew.CSB0IsPressed = FALSE;  // Force to a value for now
    UiInDataNew.CSB1IsPressed = FALSE;  // Force to a value for now
    UiInDataNew.CSB2IsPressed = FALSE;  // Force to a value for now
    UiInDataNew.CSB3IsPressed = FALSE;  // Force to a value for now
    UiInDataNew.ProxDetected  = FALSE;  // Force to a value for now

    wiced_i2c_init(&I2cPsoc);

    while (TRUE)
    {
        // Read the PSoC button state register
        PsocRegAddrBuf = PSOC_BTN_REG_ADDR;
        wiced_i2c_write(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegAddrBuf, sizeof(PsocRegAddrBuf));
        wiced_i2c_read(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegs.Btns, sizeof(PsocRegs.Btns));

        // Read the PSoC potentiometer register
        PsocRegAddrBuf = PSOC_POT_REG_ADDR;
        wiced_i2c_write(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegAddrBuf, sizeof(PsocRegAddrBuf));
        wiced_i2c_read(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegs.Pot, sizeof(PsocRegs.Pot));

        // Read the PSoC temperature register
        PsocRegAddrBuf = PSOC_TEMP_REG_ADDR;
        wiced_i2c_write(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegAddrBuf, sizeof(PsocRegAddrBuf));
        wiced_i2c_read(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegs.Temp, sizeof(PsocRegs.Temp));

        // Read the PSoC humidity register
        PsocRegAddrBuf = PSOC_HUM_REG_ADDR;
        wiced_i2c_write(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegAddrBuf, sizeof(PsocRegAddrBuf));
        wiced_i2c_read(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                    &PsocRegs.Hum, sizeof(PsocRegs.Hum));

        // Read the PSoC Light register
        PsocRegAddrBuf = PSOC_LIGHT_REG_ADDR;
        wiced_i2c_write(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                   &PsocRegAddrBuf, sizeof(PsocRegAddrBuf));
        wiced_i2c_read(&I2cPsoc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG,
                   &PsocRegs.Light, sizeof(PsocRegs.Light));

        //Read the PSoC CapSense0 Button

         //Creates temp, hum, and light buffers
         int TempAvgBuffer[TEMP_AVG_SAMPLES];
         int HumAvgBuffer[HUM_AVG_SAMPLES];
         int LightAvgBuffer[LIGHT_AVG_SAMPLES];

         //Creates Win Avg object for temp, hum, and light
         WinAvgIntType  WinTempAvgData =
         {
                 .NumSamples    = TEMP_AVG_SAMPLES,
                 .BufPtr        = TempAvgBuffer,
                 .BufIsFull     = FALSE,
                 .Div           = 0,
                 .Idx           = 0,
                 .Sum           = 0
         };

         WinAvgIntType  WinHumAvgData =
         {
                .NumSamples    = HUM_AVG_SAMPLES,
                .BufPtr        = HumAvgBuffer,
                .BufIsFull     = FALSE,
                .Div           = 0,
                .Idx           = 0,
                .Sum           = 0
         };

         WinAvgIntType  WinLightAvgData =
         {
                .NumSamples    = LIGHT_AVG_SAMPLES,
                .BufPtr        = LightAvgBuffer,
                .BufIsFull     = FALSE,
                .Div           = 0,
                .Idx           = 0,
                .Sum           = 0
         };

        // Debounce MB1
        switch(MB1State)
        {
        case MB_RELEASED:
            if (PsocRegs.Btns & PSOC_BTN_REG_MB1)
            {
                UiInDataNew.MB1IsPressed  = TRUE;
                MB1DebounceCount = MB_DEBOUNCE_COUNT;
                MB1State = MB_DEBOUNCE;
            }
            break;

        case MB_PRESSED:
            if (!(PsocRegs.Btns & PSOC_BTN_REG_MB1))
            {
                UiInDataNew.MB1IsPressed  = FALSE;
                MB1DebounceCount = MB_DEBOUNCE_COUNT;
                MB1State = MB_DEBOUNCE;
            }
            break;

        case MB_DEBOUNCE:
            if (--MB1DebounceCount <= 0)
            {
                MB1State = (PsocRegs.Btns & PSOC_BTN_REG_MB1) ? MB_PRESSED : MB_RELEASED;
            }
            break;
        }

        // Debounce MB2
        switch(MB2State)
        {
        case MB_RELEASED:
            if (PsocRegs.Btns & PSOC_BTN_REG_MB2)
            {
                UiInDataNew.MB2IsPressed  = TRUE;
                MB2DebounceCount = MB_DEBOUNCE_COUNT;
                MB2State = MB_DEBOUNCE;
            }
            break;

        case MB_PRESSED:
            if (!(PsocRegs.Btns & PSOC_BTN_REG_MB2))
            {
                UiInDataNew.MB2IsPressed  = FALSE;
                MB2DebounceCount = MB_DEBOUNCE_COUNT;
                MB2State = MB_DEBOUNCE;
            }
            break;

        case MB_DEBOUNCE:
            if (--MB2DebounceCount <= 0)
            {
                MB2State = (PsocRegs.Btns & PSOC_BTN_REG_MB2) ? MB_PRESSED : MB_RELEASED;
            }
            break;
        }

        // Determine the state of the cap sense buttons and proximity detection
        UiInDataNew.CSB0IsPressed = (PsocRegs.Btns & PSOC_BTN_REG_CSB0) ? TRUE : FALSE;
        UiInDataNew.CSB1IsPressed = (PsocRegs.Btns & PSOC_BTN_REG_CSB1) ? TRUE : FALSE;
        UiInDataNew.CSB2IsPressed = (PsocRegs.Btns & PSOC_BTN_REG_CSB2) ? TRUE : FALSE;
        UiInDataNew.CSB3IsPressed = (PsocRegs.Btns & PSOC_BTN_REG_CSB3) ? TRUE : FALSE;
        UiInDataNew.ProxDetected  = (PsocRegs.Btns & PSOC_BTN_REG_PROX) ? TRUE : FALSE;

        // Scale the temperature values to [0-100]
        UiInDataNew.Temp = (int) ((PsocRegs.Temp * PSOC_TEMP_OFFSET) + PSOC_TEMP_SCALE);
        UiInDataNew.Temp = UiInDataNew.Temp > 100 ? 100 : UiInDataNew.Temp;
        UiInDataNew.Temp = UiInDataNew.Temp <   0 ?   0 : UiInDataNew.Temp;
        UiInDataNew.Temp = WinAvgInt(UiInDataNew.Temp, &WinTempAvgData);

        // Scale the humidity values to [0-100]
        UiInDataNew.Hum = (int) (PsocRegs.Hum);
        UiInDataNew.Hum = UiInDataNew.Hum > 100 ? 100 : UiInDataNew.Hum;
        UiInDataNew.Hum = UiInDataNew.Hum <   0 ?   0 : UiInDataNew.Hum;
        UiInDataNew.Hum = WinAvgInt(UiInDataNew.Hum, &WinHumAvgData);

        // Scale the light values to [0-100]
        UiInDataNew.Light = (int) (PsocRegs.Light * PSOC_LIGHT_OFFSET);
        UiInDataNew.Light = UiInDataNew.Light > 100 ? 100 : UiInDataNew.Light;
        UiInDataNew.Light = UiInDataNew.Light <   0 ?   0 : UiInDataNew.Light;
        UiInDataNew.Light = WinAvgInt(UiInDataNew.Light, &WinLightAvgData);

        // Scale the potentiometer values to [0-100]
        UiInDataNew.Pot = (int) (PsocRegs.Pot * PSOC_POT_SCALE + PSOC_POT_OFFSET);
        UiInDataNew.Pot = UiInDataNew.Pot > 100 ? 100 : UiInDataNew.Pot;
        UiInDataNew.Pot = UiInDataNew.Pot <   0 ?   0 : UiInDataNew.Pot;

        // If any of the UI input data has changed,
        // copy the new data and inform the system manager
        if ((UiInDataNew.MB1IsPressed  != UiInData.MB1IsPressed)  ||
            (UiInDataNew.MB2IsPressed  != UiInData.MB2IsPressed)  ||
            (UiInDataNew.CSB0IsPressed != UiInData.CSB0IsPressed) ||
            (UiInDataNew.CSB1IsPressed != UiInData.CSB1IsPressed) ||
            (UiInDataNew.CSB2IsPressed != UiInData.CSB2IsPressed) ||
            (UiInDataNew.CSB3IsPressed != UiInData.CSB3IsPressed) ||
            (UiInDataNew.ProxDetected  != UiInData.ProxDetected)  ||
            (UiInDataNew.Temp          != UiInData.Temp)          ||
            (UiInDataNew.Hum           != UiInData.Hum)           ||
            (UiInDataNew.Light         != UiInData.Light)         ||
            (UiInDataNew.Pot           != UiInData.Pot))
        {
            UiInData = UiInDataNew;
            wiced_rtos_set_semaphore(&UiInSemaphoreHandle);
        }

        ToggleLed1();

        wiced_rtos_delay_milliseconds(UI_IN_SAMPLE_PERIOD);
    }
}


void UiOutThread(wiced_thread_arg_t Arg)
{
    char            StrBuffer[UI_OUT_STRBUF_SIZE];

    wiced_i2c_device_t I2cOledDisplay =
    {
        .port          = WICED_I2C_2,
        .address       = OLED_I2C_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .flags         = 0,
        .speed_mode    = I2C_HIGH_SPEED_MODE,
    };

    u8g_init_wiced_i2c_device(&I2cOledDisplay);
    u8g_InitComFn(&u8gState, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
    u8g_SetColorIndex(&u8gState, 1);

    //
    // Initialize the UI output data
    //
    UiOutData.Armed = FALSE;

    UiOutData.NewVal = 0;

    UiOutData.HighlightState = HL_NONE;

    UiOutData.TempCurVal = 40;
    UiOutData.TempLowLimit = 0;
    UiOutData.TempHighLimit = 100;

    UiOutData.HumCurVal = 50;
    UiOutData.HumLowLimit = 0;
    UiOutData.HumHighLimit = 100;

    UiOutData.LightCurVal = 60;
    UiOutData.LightLowLimit = 0;
    UiOutData.LightHighLimit = 100;

    UiOutData.TempLowAlarm = (UiOutData.TempCurVal < UiOutData.TempLowLimit);
    UiOutData.TempHighAlarm = (UiOutData.TempCurVal > UiOutData.TempHighLimit);
    UiOutData.HumLowAlarm = (UiOutData.HumCurVal < UiOutData.HumLowLimit);
    UiOutData.HumHighAlarm = (UiOutData.HumCurVal > UiOutData.HumHighLimit);
    UiOutData.LightLowAlarm = (UiOutData.LightCurVal < UiOutData.LightLowLimit);
    UiOutData.LightHighAlarm = (UiOutData.LightCurVal > UiOutData.LightHighLimit);

    UiOutData.User[0].NamePtr = "MC";
    UiOutData.User[0].IsConnected = FALSE;

    UiOutData.User[1].NamePtr = "RD";
    UiOutData.User[1].IsConnected = FALSE;

    UiOutData.User[2].NamePtr = "JE";
    UiOutData.User[2].IsConnected = FALSE;

    UiOutData.User[3].NamePtr = "SG";
    UiOutData.User[3].IsConnected = FALSE;

    UiOutData.User[4].NamePtr = "SK";
    UiOutData.User[4].IsConnected = FALSE;

    UiOutData.User[5].NamePtr = "MK";
    UiOutData.User[5].IsConnected = FALSE;

    UiOutData.User[6].NamePtr = "TL";
    UiOutData.User[6].IsConnected = FALSE;

    UiOutData.User[7].NamePtr = "EO";
    UiOutData.User[7].IsConnected = FALSE;

    UiOutData.User[8].NamePtr = "ER";
    UiOutData.User[8].IsConnected = FALSE;

    UiOutData.User[9].NamePtr = "MR";
    UiOutData.User[9].IsConnected = FALSE;

    UiOutData.User[10].NamePtr = "MS";
    UiOutData.User[10].IsConnected = FALSE;

    UiOutData.User[11].NamePtr = "JY";
    UiOutData.User[11].IsConnected = FALSE;

    while (TRUE)
    {
        // Render the display, then wait for updates
        u8g_FirstPage(&u8gState);
        do
        {
            u8g_SetFont(&u8gState, u8g_font_6x10);
            u8g_SetFontPosTop(&u8gState);

            // Render row 0
            DrawString(1, 2,  "H.E.M.P.    Armed:", FALSE);
            DrawString(109, 2, UiOutData.Armed ? "Yes" : "No ", FALSE);

            // Render row 1
            DrawString(1, 12, "New Val:", FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.NewVal);
            DrawString(49, 12, StrBuffer, TRUE);
            DrawString(74, 12, (UiOutData.Armed & UiInData.ProxDetected) ? "INTRUSION" : " ", FALSE);

            // Render row 2
            DrawString(1, 22, " Cur Low High", FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[0].IsConnected ?
                    UiOutData.User[0].NamePtr : "  ");
            DrawString(79, 22, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[4].IsConnected ?
                    UiOutData.User[4].NamePtr : "  ");
            DrawString(97, 22, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[8].IsConnected ?
                    UiOutData.User[8].NamePtr : "  ");
            DrawString(115, 22, StrBuffer, FALSE);

            // Render row 3
            DrawString(1, 32, "T", FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.TempCurVal);
            DrawString(7, 32, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.TempLowLimit);
            DrawString(31, 32, StrBuffer, UiOutData.HighlightState == HL_TEMP_LOW);
            sprintf(StrBuffer, "%c", UiOutData.TempLowAlarm ? '*' : ' ');
            DrawString(49, 32, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.TempHighLimit);
            DrawString(55, 32, StrBuffer, UiOutData.HighlightState == HL_TEMP_HIGH);
            sprintf(StrBuffer, "%c", UiOutData.TempHighAlarm ? '*' : ' ');
            DrawString(73, 32, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[1].IsConnected ?
                    UiOutData.User[1].NamePtr : "  ");
            DrawString(79, 32, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[5].IsConnected ?
                    UiOutData.User[5].NamePtr : "  ");
            DrawString(97, 32, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[9].IsConnected ?
                    UiOutData.User[9].NamePtr : "  ");
            DrawString(115, 32, StrBuffer, FALSE);

            // Render row 4
            DrawString(1, 42, "H", FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.HumCurVal);
            DrawString(7, 42, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.HumLowLimit);
            DrawString(31, 42, StrBuffer, UiOutData.HighlightState == HL_HUM_LOW);
            sprintf(StrBuffer, "%c", UiOutData.HumLowAlarm ? '*' : ' ');
            DrawString(49, 42, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.HumHighLimit);
            DrawString(55, 42, StrBuffer, UiOutData.HighlightState == HL_HUM_HIGH);
            sprintf(StrBuffer, "%c", UiOutData.HumHighAlarm ? '*' : ' ');
            DrawString(73, 42, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[2].IsConnected ?
                    UiOutData.User[2].NamePtr : "  ");
            DrawString(79, 42, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[6].IsConnected ?
                    UiOutData.User[6].NamePtr : "  ");
            DrawString(97, 42, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[10].IsConnected ?
                    UiOutData.User[10].NamePtr : "  ");
            DrawString(115, 42, StrBuffer, FALSE);

            // Render row 5
            DrawString(1, 52, "L", FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.LightCurVal);
            DrawString(7, 52, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.LightLowLimit);
            DrawString(31, 52, StrBuffer, UiOutData.HighlightState == HL_LIGHT_LOW);
            sprintf(StrBuffer, "%c", UiOutData.LightLowAlarm ? '*' : ' ');
            DrawString(49, 52, StrBuffer, FALSE);
            sprintf(StrBuffer, "%3d", UiOutData.LightHighLimit);
            DrawString(55, 52, StrBuffer, UiOutData.HighlightState == HL_LIGHT_HIGH);
            sprintf(StrBuffer, "%c", UiOutData.LightHighAlarm ? '*' : ' ');
            DrawString(73, 52, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[3].IsConnected ?
                    UiOutData.User[3].NamePtr : "  ");
            DrawString(79, 52, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[7].IsConnected ?
                    UiOutData.User[7].NamePtr : "  ");
            DrawString(97, 52, StrBuffer, FALSE);
            sprintf(StrBuffer, "%.2s", UiOutData.User[11].IsConnected ?
                    UiOutData.User[11].NamePtr : "  ");
            DrawString(115, 52, StrBuffer, FALSE);

        } while (u8g_NextPage(&u8gState));

        wiced_rtos_get_semaphore(&UiOutSemaphoreHandle, WICED_WAIT_FOREVER);
    }
}


void DrawString(int XPos, int YPos, char * StrPtr, Bool Invert)
{
    if (Invert)
    {
        u8g_DrawBox(&u8gState, XPos, YPos, strlen(StrPtr) * FONT_WIDTH, FONT_HEIGHT);
        u8g_SetColorIndex(&u8gState, 0);
    }

    u8g_DrawStr(&u8gState, XPos, YPos, StrPtr);
    u8g_SetColorIndex(&u8gState, 1);
}


void ToggleLed1(void)
{
    static Bool Led1IsOn = FALSE;

    if (Led1IsOn)
    {
        Led1IsOn = FALSE;
        wiced_gpio_output_low(WICED_LED1);
    }
    else
    {
        Led1IsOn = TRUE;
        wiced_gpio_output_high(WICED_LED1);
    }
}

int WinAvgInt(int NewVal, WinAvgIntType * WinAvgDataPtr)
{
    if (WinAvgDataPtr->BufIsFull)
    {
        WinAvgDataPtr->Sum -= WinAvgDataPtr->BufPtr[WinAvgDataPtr->Idx];
    }

    WinAvgDataPtr->Sum += NewVal;
    WinAvgDataPtr->BufPtr[WinAvgDataPtr->Idx] = NewVal;

    if(++(WinAvgDataPtr->Div) >= WinAvgDataPtr->NumSamples)
    {
        WinAvgDataPtr->Div = WinAvgDataPtr->NumSamples;
        WinAvgDataPtr->BufIsFull = TRUE;
    }

    if(++(WinAvgDataPtr->Idx) == WinAvgDataPtr->NumSamples)
    {
        WinAvgDataPtr->Idx = 0;
    }

    return WinAvgDataPtr->Sum / WinAvgDataPtr->Div;
}
