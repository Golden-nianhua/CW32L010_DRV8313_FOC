#include "main.h"
#include "math.h"
#include "pid.h"
#include "foc.h"
#include "drv8313.h"
#include "mt6701.h"
#include "position_sensor.h"
#include "current.h"

void SYS_Init(void);
void ShowDigitalsign(void);

volatile uint8_t gFlagIrq;
volatile uint32_t microseconds = 0;
float motor_target = 0.0f;

#define FOC_VBUS 12.0f
#define FOC_PP   7
#define FOC_DIR  1

// FOC硬件抽象层回调函数
void foc_pwm_callback(float dc_a, float dc_b, float dc_c) {
    DRV8313_UA_PWM(dc_a);
    DRV8313_UB_PWM(dc_b);
    DRV8313_UC_PWM(dc_c);
}

void foc_enable_callback(void) {
    DRV8313_ENABLE();
}

void foc_disable_callback(void) {
    DRV8313_DISABLE();
}

// ADC 读取回调函数实现（阻塞式采样）
void ADC_ReadCurrentSensor(uint16_t *adc_value_a, uint16_t *adc_value_b) {
    // 启动ADC转换
    ADC_SoftwareStartConvCmd(ENABLE);

    // 阻塞等待转换完成（检查 EOS 标志：End of Sequence）
    while (!(gFlagIrq & ADC_ISR_EOS_Msk)) {}

    *adc_value_a = ADC_GetConversionValue(0);
    *adc_value_b = ADC_GetConversionValue(1);

    gFlagIrq = 0;
}


int main(void) {

    SYS_Init();

    uint16_t k=0,j = 0;

    // 创建位置传感器实例并初始化
    PositionSensor position_sensor = {0};
    PositionSensor_Init(&position_sensor, MT6701_RESOLUTION, MT6701_GetRawAngle);

    // 参数：采样电阻(Ω), 放大器增益, ADC参考电压(V), ADC分辨率(bits)
    CurrentSensor current_sensor = {0};
    CurrentSensor_Init(&current_sensor, 0.01f, 50.0f, 3.3f, 12, ADC_ReadCurrentSensor);

    FOCMotor motor = {0};
    FOC_Init(&motor, &position_sensor, &current_sensor, FOC_VBUS, FOC_PP, FOC_DIR,
             foc_pwm_callback, foc_enable_callback, foc_disable_callback);

    FOC_AlignSensor(&motor);
    CurrentSensor_Calibrate(&current_sensor, 100);

    //位置闭环 角度-速度-力
    FOC_SET_VELOCITY_PID(&motor, 0.02f, 1.0f, 0.0f, 100000.0f, 0.5);
    FOC_SET_POSITION_PID(&motor, 1.0f, 0.f, 0.0f, 100000.0f, 30.0f);

    /*//力位闭环
    FOC_SET_POSITION_PID(&motor, 0.133f, 0.f, 0.0f, 100000.0f, 100);*/

    /*//速度闭环
    FOC_SET_VELOCITY_PID(&motor, 0.005f, 0.0f, 0.0f, 0.0f, FOC_VBUS/2);*/

    /*//电流闭环
    FOC_SET_CURRENT_Q_PID(&motor, 5.0f, 200.f, 0.0f, 100000.0f, FOC_VBUS);*/

    while (1) {
        //更新传感器数据
        CurrentSensor_Update(&current_sensor);
        float Iq = -FOC_GET_IQ(&motor);
        PositionSensor_Update(motor.sensor);
        float vel = FOC_GET_VELOCITY(&motor);
        float angle = (float)motor.DIR * PositionSensor_GetAngle(motor.sensor);

        //位置闭环 角度-速度-力
        float pid = PID_Calculate(&motor.pid_velocity, PID_Calculate(&motor.pid_position, (motor_target-angle)*180/PI)-vel);

        /*//力位闭环
        float pid = PID_Calculate(&motor.pid_position, (motor_target-angle)*180/PI);*/

        /*//速度闭环
        float pid = PID_Calculate(&motor.pid_velocity, (motor_target-vel)*180/PI);*/

        /*//电流闭环
        float pid = PID_Calculate(&motor.pid_current_q, (motor_target-Iq));*/

        FOC_SetTorque(&motor, pid, FOC_GetElectricalAngle(&motor));

        //发送@<angle>;设置角度
         k++;
         if (k >= 1000) {
             k = 0;
             LOG("t: %.3f, a: %.3f, v: %.3f, I: %.3f, p: %.3f\r\n", motor_target, angle, vel, Iq, pid);
         }
    }
}

void SYS_Init(void) {
    rcc_Init();
    led_Init();
    LED_ON();
    LED_SET(100);
    SysTickDelay(250);
    LED_SET(25);
    SysTickDelay(250);
    LED_SET(0);

#ifdef ENABLE_LOG
    uart_Init();
    ShowDigitalsign();
#endif

    key_Init();
    i2c_Init();
    adc_Init();
    atim_Init();
    btim1_Init();
}

void GPIOA_IRQHandlerCallback(void) {
    if (CW_GPIOA->ISR_f.PIN3) {
        GPIOA_INTFLAG_CLR(bv3);
        if (CW_GPIOA->IDR & GPIO_PIN_3) {
            PB01_SETLOW();
            LED_SET(0);
        } else {
            PB01_SETHIGH();
            LED_SET(100);
        }
    }
}


void ShowDigitalsign(void) {
    uint8_t Chip_Type[24];
    uint8_t Pin_Count;
    uint32_t Flash_Size;
    uint16_t Ram_Size;
    uint8_t Chip_Uid[10];

    DIGITALSIGN_GetChipType(Chip_Type);
    Pin_Count = DIGITALSIGN_GetPinCount();
    Flash_Size = DIGITALSIGN_GetFlashSize();
    Ram_Size = DIGITALSIGN_GetRamSize();
    DIGITALSIGN_GetChipUid(Chip_Uid);

    LOG("Chip Type: %s\r\n", Chip_Type);
    LOG("Pin Count: %d\r\n", Pin_Count);
    LOG("Flash Size: %d B\r\n", Flash_Size);
    LOG("RAM Size: %d B\r\n", Ram_Size);
    LOG("Chip UID: ");
    for (int i = 0; i < 10; i++) {
        LOG("%02X", Chip_Uid[i]);
    }
    LOG("\r\n");
}

int __io_putchar(const int ch) {
    UART_SendData_8bit(CW_UART1, ch);
    while (UART_GetFlagStatus(CW_UART1, UART_FLAG_TXE) == RESET) {}
    return ch;
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif
