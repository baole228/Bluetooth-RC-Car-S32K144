#include "sdk_project_config.h"
#include "ftm_pwm_driver.h"
#include "lpuart_driver.h"

#include "peripherals_flexTimer_pwm_1.h"
#include "peripherals_lpuart_1.h"

volatile int exit_code = 0;

/* =========================
   FTM2 Instance
   ========================= */
#ifndef INST_FLEXTIMER_PWM_1
#define INST_FLEXTIMER_PWM_1    2U
#endif

/* =========================
   FTM2 Channel
   ========================= */
#define SERVO_CHANNEL           0U      /* FTM2_CH0 -> PTD10 */
#define MOTOR_CHANNEL           3U      /* FTM2_CH3 -> PTD5  */

/* =========================
   FTM2 = 50 Hz
   Clock = 48 MHz, Prescaler /16
   Counter = 3 MHz
   Period  = 60000 ticks
   ========================= */
#define PWM_PERIOD_TICKS        60000U

/* =========================
   Servo pulse width (ticks)
   1.25ms = 3750  -> phai 45
   1.50ms = 4500  -> giua
   1.75ms = 5250  -> trai 45
   ========================= */
#define SERVO_LEFT_45_TICKS     5700U
#define SERVO_CENTER_TICKS      4500U
#define SERVO_RIGHT_45_TICKS    3300U

/* =========================
   Motor PWM duty (ticks)
   ========================= */
#define MOTOR_DUTY_0            0U
#define MOTOR_DUTY_25           15000U
#define MOTOR_DUTY_50           30000U
#define MOTOR_DUTY_75           45000U
#define MOTOR_DUTY_100          60000U

#define MOTOR_RUN_DUTY          MOTOR_DUTY_75

/* =========================
   L298N Direction Pins
   ========================= */
#define L298_IN1_PORT           PTD
#define L298_IN1_PIN            17U

#define L298_IN2_PORT           PTE
#define L298_IN2_PIN            0U

/* =========================
   UART config
   ========================= */
#define UART_RX_TIMEOUT_MS      20U

/* =========================
   Servo ve giua sau bao nhieu
   lan timeout lien tiep:
   20ms x 3 = ~60ms sau nha nut
   ========================= */
#define SERVO_HOLD_CYCLES       3U

/* FTM runtime state */
static ftm_state_t flexTimer_pwm_1_State;

/* =========================
   Delay don gian (~1ms/loop)
   ========================= */
static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t count;
    while (ms--)
    {
        count = 4000U;
        while (count--) {}
    }
}

/* =========================
   Motor direction
   ========================= */
static void motor_forward(void)
{
    PINS_DRV_WritePin(L298_IN1_PORT, L298_IN1_PIN, 1U);
    PINS_DRV_WritePin(L298_IN2_PORT, L298_IN2_PIN, 0U);
}

static void motor_reverse(void)
{
    PINS_DRV_WritePin(L298_IN1_PORT, L298_IN1_PIN, 0U);
    PINS_DRV_WritePin(L298_IN2_PORT, L298_IN2_PIN, 1U);
}

static void motor_stop(void)
{
    PINS_DRV_WritePin(L298_IN1_PORT, L298_IN1_PIN, 0U);
    PINS_DRV_WritePin(L298_IN2_PORT, L298_IN2_PIN, 0U);
}

/* =========================
   Update PWM theo ticks
   ========================= */
static void ftm2_update_pwm_ticks(uint8_t channel, uint16_t ticks)
{
    if (ticks > PWM_PERIOD_TICKS)
    {
        ticks = (uint16_t)PWM_PERIOD_TICKS;
    }
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
                             channel,
                             FTM_PWM_UPDATE_IN_TICKS,
                             ticks,
                             0U,
                             true);
}

/* =========================
   UART recover
   ========================= */
static void uart_recover(void)
{
    LPUART_DRV_AbortReceivingData(INST_LPUART_1);
    LPUART_DRV_AbortSendingData(INST_LPUART_1);
    LPUART_DRV_Deinit(INST_LPUART_1);
    delay_ms(5U);
    (void)LPUART_DRV_Init(INST_LPUART_1,
                           &lpUartState2,
                           &lpuart_2_InitConfig0);
}

/* =========================
   Servo control
   ========================= */
static void servo_left(void)
{
    ftm2_update_pwm_ticks(SERVO_CHANNEL, SERVO_LEFT_45_TICKS);
}

static void servo_right(void)
{
    ftm2_update_pwm_ticks(SERVO_CHANNEL, SERVO_RIGHT_45_TICKS);
}

static void servo_center(void)
{
    ftm2_update_pwm_ticks(SERVO_CHANNEL, SERVO_CENTER_TICKS);
}

/* =========================
   Motor control
   ========================= */
static void motor_run_forward(void)
{
    motor_forward();
    ftm2_update_pwm_ticks(MOTOR_CHANNEL, MOTOR_RUN_DUTY);
    PINS_DRV_TogglePins(PTD, (1 << 15));   // LED debug
}

static void motor_run_backward(void)
{
    motor_reverse();
    ftm2_update_pwm_ticks(MOTOR_CHANNEL, MOTOR_RUN_DUTY);
}

static void motor_run_stop(void)
{
    motor_stop();
    ftm2_update_pwm_ticks(MOTOR_CHANNEL, MOTOR_DUTY_0);
}

/* =========================
   Kiem tra lenh hop le
   ========================= */
static bool is_valid_command(uint8_t cmd)
{
    return (cmd == 'F' || cmd == 'f' ||
            cmd == 'B' || cmd == 'b' ||
            cmd == 'L' || cmd == 'l' ||
            cmd == 'R' || cmd == 'r' ||
            cmd == 'S' || cmd == 's' ||
            cmd == 'G' || cmd == 'g' ||   /* tien + trai */
            cmd == 'I' || cmd == 'i' ||   /* tien + phai */
            cmd == 'H' || cmd == 'h' ||   /* lui  + trai */
            cmd == 'J' || cmd == 'j');    /* lui  + phai */
}

/* =========================
   Chuan hoa chu hoa
   ========================= */
static uint8_t normalize_cmd(uint8_t cmd)
{
    if (cmd >= 'a' && cmd <= 'z')
    {
        return (uint8_t)(cmd - 32U);
    }
    return cmd;
}

/* =========================
   Main
   ========================= */
int main(void)
{
    status_t status         = STATUS_SUCCESS;
    uint8_t  rxData         = 0U;
    uint8_t  errorCount     = 0U;

    /*
     * State servo va motor doc lap:
     *   activeServo    : lenh servo dang giu (0=giua, 'L', 'R')
     *   activeMotor    : lenh motor dang giu (0=dung, 'F', 'B')
     *   servoIdleCount : dem timeout lien tiep khong co lenh L/R
     *   servoAtCenter  : chi goi PWM servo ve giua dung 1 lan
     */
    uint8_t activeServo     = 0U;
    uint8_t activeMotor     = 0U;
    uint8_t servoIdleCount  = 0U;
    bool    servoAtCenter   = true;
    uint8_t lostSignalCount = 0U;

    /* 1. Init clock */
    status = CLOCK_DRV_Init(&clockMan1_InitConfig0);
    DEV_ASSERT(status == STATUS_SUCCESS);

    /* 2. Init pin mux */
    status = PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0,
                           g_pin_mux_InitConfigArr0);
    DEV_ASSERT(status == STATUS_SUCCESS);

    /* 3. Init FTM2 */
    status = FTM_DRV_Init(INST_FLEXTIMER_PWM_1,
                          &flexTimer_pwm_1_InitConfig,
                          &flexTimer_pwm_1_State);
    DEV_ASSERT(status == STATUS_SUCCESS);

    status = FTM_DRV_InitPwm(INST_FLEXTIMER_PWM_1,
                             &flexTimer_pwm_1_PwmConfig);
    DEV_ASSERT(status == STATUS_SUCCESS);

    /* 4. Init LPUART2 (HC-05, 9600 8N1) */
    status = LPUART_DRV_Init(INST_LPUART_1,
                             &lpUartState2,
                             &lpuart_2_InitConfig0);
    DEV_ASSERT(status == STATUS_SUCCESS);

    /* 5. Trang thai ban dau */
    motor_run_stop();
    servo_center();

    /* ========================
       Main loop
       ======================== */
    while (1)
    {
        rxData = 0U;

        status = LPUART_DRV_ReceiveDataBlocking(INST_LPUART_1,
                                                &rxData,
                                                1U,
                                                UART_RX_TIMEOUT_MS);

        if (status == STATUS_SUCCESS)
        {
            errorCount = 0U;
            lostSignalCount = 0U;      // Reset khi nhận được dữ liệu

            if (is_valid_command(rxData))
            {
                uint8_t cmd = normalize_cmd(rxData);

                switch (cmd)
                {
                    /* --- CHI TIEN --- */
                    case 'F':
                        if (activeMotor != 'F')
                        {
                            activeMotor = 'F';
                            motor_run_forward();
                            PINS_DRV_TogglePins(PTD, (1 << 15));
                        }
                        break;

                    /* --- CHI LUI --- */
                    case 'B':
                        if (activeMotor != 'B')
                        {
                            activeMotor = 'B';
                            motor_run_backward();
                        }
                        break;

                    /* --- CHI RE TRAI --- */
                    case 'L':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeServo != 'L')
                        {
                            activeServo = 'L';
                            servo_left();
                        }
                        break;

                    /* --- CHI RE PHAI --- */
                    case 'R':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeServo != 'R')
                        {
                            activeServo = 'R';
                            servo_right();
                        }
                        break;

                    /* --- TIEN + TRAI --- */
                    case 'G':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeMotor != 'F')
                        {
                            activeMotor = 'F';
                            motor_run_forward();
                        }
                        if (activeServo != 'L')
                        {
                            activeServo = 'L';
                            servo_left();
                        }
                        break;

                    /* --- TIEN + PHAI --- */
                    case 'I':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeMotor != 'F')
                        {
                            activeMotor = 'F';
                            motor_run_forward();
                        }
                        if (activeServo != 'R')
                        {
                            activeServo = 'R';
                            servo_right();
                        }
                        break;

                    /* --- LUI + TRAI --- */
                    case 'H':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeMotor != 'B')
                        {
                            activeMotor = 'B';
                            motor_run_backward();
                        }
                        if (activeServo != 'L')
                        {
                            activeServo = 'L';
                            servo_left();
                        }
                        break;

                    /* --- LUI + PHAI --- */
                    case 'J':
                        servoIdleCount = 0U;
                        servoAtCenter  = false;
                        if (activeMotor != 'B')
                        {
                            activeMotor = 'B';
                            motor_run_backward();
                        }
                        if (activeServo != 'R')
                        {
                            activeServo = 'R';
                            servo_right();
                        }
                        break;

                    /* --- STOP: dung ca hai --- */
                    case 'S':
                        activeMotor    = 0U;
                        activeServo    = 0U;
                        servoIdleCount = SERVO_HOLD_CYCLES;
                        servoAtCenter  = true;
                        motor_run_stop();
                        servo_center();
                        break;

                    default:
                        break;
                }
            }
        }
        else
        {
            if (status != STATUS_TIMEOUT)
            {
                /*
                 * Loi UART that su (busy, overrun...):
                 * abort de unlock driver.
                 * >= 5 loi lien tiep -> reinit hoan toan.
                 */
                LPUART_DRV_AbortReceivingData(INST_LPUART_1);
                errorCount++;

                if (errorCount >= 5U)
                {
                    errorCount = 0U;
                    uart_recover();
                }
            }
            else
            {
                lostSignalCount++;

                /* Mất Bluetooth khoảng 400ms thì dừng xe */
                if (lostSignalCount >= 20U)
                {
                    activeMotor = 0U;
                    motor_run_stop();

                    activeServo = 0U;

                    if (!servoAtCenter)
                    {
                        servo_center();
                        servoAtCenter = true;
                    }
                }
                else
                {
                    if (servoIdleCount < SERVO_HOLD_CYCLES)
                    {
                        servoIdleCount++;
                    }

                    if (servoIdleCount >= SERVO_HOLD_CYCLES && !servoAtCenter)
                    {
                        activeServo = 0U;
                        servoAtCenter = true;
                        servo_center();
                    }
                }
            }

        } delay_ms(2U);
    }

    return exit_code;
}
