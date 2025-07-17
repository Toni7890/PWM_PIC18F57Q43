/*
 * =============================================================================
 * MAIN.C - CONTROL DE 2 MOTORES DC CON L298N Y REDUCTORES 180:1
 * =============================================================================
 * 
 * Microcontrolador: PIC18F57Q43
 * Hardware:
 * - PIC18F57Q43 Curiosity Nano
 * - Driver L298N
 * - 2x Micro Motores con Reductor 180:1 (3-6V, Pl�stico Amarillo)
 * - Fuente: 12V 2A
 * 
 * Especificaciones de los motores:
 * - Voltaje operaci�n: 3-6V (�PTIMO: 4.5V para 50% de 9V efectivo)
 * - Reducci�n: 180:1 (alto torque, baja velocidad)
 * - Material: Pl�stico, Color: Amarillo
 * - N�mero de ejes: 2
 * 
 * Conexiones:
 * Motor A (Izquierdo):
 * RC2 (CCP1/PWM) -> ENA (L298N)  - Velocidad Motor A
 * RA0 (GPIO)     -> IN1 (L298N)  - Direcci�n Motor A
 * RA1 (GPIO)     -> IN2 (L298N)  - Direcci�n Motor A
 * 
 * Motor B (Derecho):
 * RC1 (CCP2/PWM) -> ENB (L298N)  - Velocidad Motor B
 * RA2 (GPIO)     -> IN3 (L298N)  - Direcci�n Motor B
 * RA3 (GPIO)     -> IN4 (L298N)  - Direcci�n Motor B
 * 
 * =============================================================================
 */

#include "mcc_generated_files/system/system.h"
#include <xc.h>

// =============================================================================
// DEFINICIONES PARA CONTROL DE 2 MOTORES
// =============================================================================

// Control Motor A (pines RA0 y RA1)
#define MOTOR_A_IN1_SetHigh()   do { LATAbits.LATA0 = 1; } while(0)
#define MOTOR_A_IN1_SetLow()    do { LATAbits.LATA0 = 0; } while(0)
#define MOTOR_A_IN2_SetHigh()   do { LATAbits.LATA1 = 1; } while(0)
#define MOTOR_A_IN2_SetLow()    do { LATAbits.LATA1 = 0; } while(0)

// Control Motor B (pines RA2 y RA3)
#define MOTOR_B_IN3_SetHigh()   do { LATAbits.LATA2 = 1; } while(0)
#define MOTOR_B_IN3_SetLow()    do { LATAbits.LATA2 = 0; } while(0)
#define MOTOR_B_IN4_SetHigh()   do { LATAbits.LATA3 = 1; } while(0)
#define MOTOR_B_IN4_SetLow()    do { LATAbits.LATA3 = 0; } while(0)

// Estados del motor
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} motor_direction_t;

// Identificadores de motor
typedef enum {
    MOTOR_A = 0,  // Motor Izquierdo
    MOTOR_B = 1   // Motor Derecho  
} motor_id_t;

// Configuraciones espec�ficas para motores con reductor 180:1
#define MOTOR_MIN_SPEED_REDUCTION   15    // M�nimo 15% para vencer la reducci�n
#define MOTOR_MAX_SPEED_SAFE        85    // M�ximo 85% para proteger reductores
#define MOTOR_OPTIMAL_VOLTAGE_PCT   38    // 38% = ~4.5V con 12V fuente
#define MOTOR_TORQUE_SPEED          50    // 50% para m�ximo torque

// Configuraciones para arranque sincronizado
#define MOTOR_STARTUP_SPEED         70    // Velocidad alta inicial para arranque r�pido
#define MOTOR_RUNNING_SPEED         38    // Velocidad constante de operaci�n
#define STARTUP_DURATION_MS         300   // Duraci�n del arranque r�pido (300ms)
#define SYNC_DELAY_BETWEEN_MOTORS   5     // Delay m�nimo entre motores (5ms)

// Variables globales para ambos motores
volatile uint8_t motor_speeds[2] = {0, 0};
volatile motor_direction_t motor_directions[2] = {MOTOR_STOP, MOTOR_STOP};

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================

void delay_seconds(uint8_t seconds);
void delay_half_second(void);

// Funciones b�sicas
void Motors_Initialize(void);
void Motor_SetSpeed(motor_id_t motor_id, uint8_t speed_percent);
void Motor_SetDirection(motor_id_t motor_id, motor_direction_t direction);
void Motor_Control(motor_id_t motor_id, motor_direction_t direction, uint8_t speed_percent);
void Motor_Stop(motor_id_t motor_id);
void Motor_Brake(motor_id_t motor_id);
void Motors_StopAll(void);

// Funciones para reductores 180:1
void Motor_SetOptimalSpeed(motor_id_t motor_id, motor_direction_t direction);
void Motor_SetTorqueSpeed(motor_id_t motor_id, motor_direction_t direction);
void Motor_StartSlow(motor_id_t motor_id, motor_direction_t direction);

// Funciones de sincronizaci�n y arranque r�pido
void Motors_SynchronizedStart(motor_direction_t dir_a, motor_direction_t dir_b, uint8_t final_speed);
void Motors_FastStart_ThenConstant(motor_direction_t dir_a, motor_direction_t dir_b);
void Motors_SynchronizedStop(void);
void Motors_SynchronizedDirection(motor_direction_t dir_a, motor_direction_t dir_b, uint8_t speed);

// Funciones de robot
void Robot_MoveForward(uint8_t speed);
void Robot_MoveBackward(uint8_t speed);
void Robot_TurnLeft(uint8_t speed);
void Robot_TurnRight(uint8_t speed);
void Robot_RotateLeft(uint8_t speed);
void Robot_RotateRight(uint8_t speed);
void Robot_Stop(void);

// Secuencias de demostraci�n
void Demo_BasicMovements(void);
void Demo_PrecisionControl(void);
void Demo_TorqueTest(void);

uint16_t ConvertPercentToPWM(uint8_t percentage);

// =============================================================================
// FUNCI�N PRINCIPAL
// =============================================================================

int main(void)
{
    // Inicializaci�n del sistema MCC
    SYSTEM_Initialize();
    
    // Inicializar control de motores
    Motors_Initialize();
    
    // Habilitar interrupciones globales (opcional)
    // INTERRUPT_GlobalInterruptHighEnable(); 
    // INTERRUPT_GlobalInterruptLowEnable(); 

    // Mensaje inicial
    // printf("\r\n=== CONTROL 2 MOTORES CON REDUCTOR 180:1 ===\r\n");
    
    // Loop principal - Ciclo entre diferentes demos
    uint8_t demo_mode = 0;
    
    while(1)
    {
        switch(demo_mode) {
            case 0:
                Demo_BasicMovements();
                break;
            case 1:
                Demo_PrecisionControl(); 
                break;
            case 2:
                Demo_TorqueTest();
                break;
            default:
                demo_mode = 0;
                continue;
        }
        
        demo_mode++;
        if (demo_mode > 2) demo_mode = 0;
        
        // Pausa entre demos
        delay_seconds(3);
    }
    
    return 0;
}

// =============================================================================
// FUNCIONES DE DELAY
// =============================================================================

void delay_seconds(uint8_t seconds)
{
    for (uint8_t i = 0; i < seconds; i++) {
        __delay_ms(1000);
    }
}

void delay_half_second(void)
{
    __delay_ms(500);
}

// =============================================================================
// FUNCIONES DE CONTROL DE MOTORES
// =============================================================================

/**
 * @brief Inicializa el control de ambos motores
 */
void Motors_Initialize(void)
{
    // Configurar todos los pines de direcci�n como salidas digitales
    TRISAbits.TRISA0 = 0;  // RA0 Motor A IN1
    TRISAbits.TRISA1 = 0;  // RA1 Motor A IN2
    TRISAbits.TRISA2 = 0;  // RA2 Motor B IN3
    TRISAbits.TRISA3 = 0;  // RA3 Motor B IN4
    
    // Configurar pines en modo digital
    ANSELAbits.ANSELA0 = 0;  // RA0 digital
    ANSELAbits.ANSELA1 = 0;  // RA1 digital
    ANSELAbits.ANSELA2 = 0;  // RA2 digital
    ANSELAbits.ANSELA3 = 0;  // RA3 digital
    
    // Estado inicial: ambos motores detenidos
    Motors_StopAll();
    
    // Configurar PWM inicial en 0% para ambos motores
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

/**
 * @brief Establece la velocidad de un motor espec�fico
 */
void Motor_SetSpeed(motor_id_t motor_id, uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    
    // Para motores con reductor, aplicar velocidad m�nima si no es 0
    if (speed_percent > 0 && speed_percent < MOTOR_MIN_SPEED_REDUCTION) {
        speed_percent = MOTOR_MIN_SPEED_REDUCTION;
    }
    
    // Limitar velocidad m�xima para proteger reductores
    if (speed_percent > MOTOR_MAX_SPEED_SAFE) {
        speed_percent = MOTOR_MAX_SPEED_SAFE;
    }
    
    uint16_t pwm_value = ConvertPercentToPWM(speed_percent);
    
    if (motor_id == MOTOR_A) {
        CCP1_LoadDutyValue(pwm_value);
    } else if (motor_id == MOTOR_B) {
        CCP2_LoadDutyValue(pwm_value);
    }
    
    motor_speeds[motor_id] = speed_percent;
}

/**
 * @brief Establece la direcci�n de un motor espec�fico
 */
void Motor_SetDirection(motor_id_t motor_id, motor_direction_t direction)
{
    if (motor_id == MOTOR_A) {
        switch (direction) {
            case MOTOR_FORWARD:
                MOTOR_A_IN1_SetHigh();
                MOTOR_A_IN2_SetLow();
                break;
            case MOTOR_BACKWARD:
                MOTOR_A_IN1_SetLow();
                MOTOR_A_IN2_SetHigh();
                break;
            case MOTOR_BRAKE:
                MOTOR_A_IN1_SetHigh();
                MOTOR_A_IN2_SetHigh();
                break;
            case MOTOR_STOP:
            default:
                MOTOR_A_IN1_SetLow();
                MOTOR_A_IN2_SetLow();
                break;
        }
    } else if (motor_id == MOTOR_B) {
        switch (direction) {
            case MOTOR_FORWARD:
                MOTOR_B_IN3_SetHigh();
                MOTOR_B_IN4_SetLow();
                break;
            case MOTOR_BACKWARD:
                MOTOR_B_IN3_SetLow();
                MOTOR_B_IN4_SetHigh();
                break;
            case MOTOR_BRAKE:
                MOTOR_B_IN3_SetHigh();
                MOTOR_B_IN4_SetHigh();
                break;
            case MOTOR_STOP:
            default:
                MOTOR_B_IN3_SetLow();
                MOTOR_B_IN4_SetLow();
                break;
        }
    }
    
    motor_directions[motor_id] = direction;
}

/**
 * @brief Control completo de un motor (direcci�n + velocidad)
 */
void Motor_Control(motor_id_t motor_id, motor_direction_t direction, uint8_t speed_percent)
{
    // Si hay cambio de direcci�n y motor est� en movimiento, parar primero
    if (motor_directions[motor_id] != direction && motor_speeds[motor_id] > 0) {
        Motor_Stop(motor_id);
        __delay_ms(200);  // Pausa m�s larga para reductores
    }
    
    Motor_SetDirection(motor_id, direction);
    
    if (direction == MOTOR_STOP || direction == MOTOR_BRAKE) {
        Motor_SetSpeed(motor_id, 0);
    } else {
        Motor_SetSpeed(motor_id, speed_percent);
    }
}

/**
 * @brief Detiene un motor espec�fico
 */
void Motor_Stop(motor_id_t motor_id)
{
    Motor_SetSpeed(motor_id, 0);
    Motor_SetDirection(motor_id, MOTOR_STOP);
}

/**
 * @brief Frena un motor espec�fico
 */
void Motor_Brake(motor_id_t motor_id)
{
    Motor_SetDirection(motor_id, MOTOR_BRAKE);
    Motor_SetSpeed(motor_id, 100);
    __delay_ms(150);  // Tiempo de frenado
    Motor_Stop(motor_id);
}

/**
 * @brief Detiene ambos motores
 */
void Motors_StopAll(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

// =============================================================================
// FUNCIONES DE SINCRONIZACI�N Y ARRANQUE R�PIDO
// =============================================================================

/**
 * @brief Arranque r�pido sincronizado: alta velocidad inicial + velocidad constante
 * @param dir_a Direcci�n motor A
 * @param dir_b Direcci�n motor B
 */
void Motors_FastStart_ThenConstant(motor_direction_t dir_a, motor_direction_t dir_b)
{
    // FASE 1: Configurar direcciones SIMULT�NEAMENTE
    Motor_SetDirection(MOTOR_A, dir_a);
    Motor_SetDirection(MOTOR_B, dir_b);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);  // 5ms para estabilizar
    
    // FASE 2: Arranque r�pido SIMULT�NEO para vencer inercia
    Motor_SetSpeed(MOTOR_A, MOTOR_STARTUP_SPEED);  // 70% inicial
    Motor_SetSpeed(MOTOR_B, MOTOR_STARTUP_SPEED);  // 70% inicial
    
    // FASE 3: Mantener velocidad alta por per�odo corto
    __delay_ms(STARTUP_DURATION_MS);  // 300ms de arranque r�pido
    
    // FASE 4: Reducir a velocidad constante SIMULT�NEAMENTE
    Motor_SetSpeed(MOTOR_A, MOTOR_RUNNING_SPEED);  // 38% constante
    Motor_SetSpeed(MOTOR_B, MOTOR_RUNNING_SPEED);  // 38% constante
}

/**
 * @brief Arranque sincronizado con velocidad personalizable
 * @param dir_a Direcci�n motor A
 * @param dir_b Direcci�n motor B  
 * @param final_speed Velocidad final deseada
 */
void Motors_SynchronizedStart(motor_direction_t dir_a, motor_direction_t dir_b, uint8_t final_speed)
{
    // Configurar direcciones simult�neamente
    Motor_SetDirection(MOTOR_A, dir_a);
    Motor_SetDirection(MOTOR_B, dir_b);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);
    
    // Arranque r�pido para vencer inercia
    Motor_SetSpeed(MOTOR_A, MOTOR_STARTUP_SPEED);
    Motor_SetSpeed(MOTOR_B, MOTOR_STARTUP_SPEED);
    __delay_ms(STARTUP_DURATION_MS);
    
    // Transici�n suave a velocidad final
    uint8_t current_speed = MOTOR_STARTUP_SPEED;
    while (current_speed != final_speed) {
        if (current_speed > final_speed) {
            current_speed -= 2;
            if (current_speed < final_speed) current_speed = final_speed;
        } else {
            current_speed += 2;
            if (current_speed > final_speed) current_speed = final_speed;
        }
        
        Motor_SetSpeed(MOTOR_A, current_speed);
        Motor_SetSpeed(MOTOR_B, current_speed);
        __delay_ms(20);  // 20ms entre pasos para transici�n suave
    }
}

/**
 * @brief Parada sincronizada de ambos motores
 */
void Motors_SynchronizedStop(void)
{
    // Parada simult�nea para evitar desfase
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
    
    // Esperar un ciclo para sincronizar
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);
    
    // Configurar direcciones en STOP
    Motor_SetDirection(MOTOR_A, MOTOR_STOP);
    Motor_SetDirection(MOTOR_B, MOTOR_STOP);
}

/**
 * @brief Control direccional sincronizado
 * @param dir_a Direcci�n motor A
 * @param dir_b Direcci�n motor B
 * @param speed Velocidad para ambos motores
 */
void Motors_SynchronizedDirection(motor_direction_t dir_a, motor_direction_t dir_b, uint8_t speed)
{
    // Si hay cambio de direcci�n, parar ambos primero
    bool direction_change = (motor_directions[MOTOR_A] != dir_a && motor_speeds[MOTOR_A] > 0) ||
                           (motor_directions[MOTOR_B] != dir_b && motor_speeds[MOTOR_B] > 0);
    
    if (direction_change) {
        Motors_SynchronizedStop();
        __delay_ms(100);  // Pausa de seguridad
    }
    
    // Aplicar nuevas direcciones y velocidad
    Motors_SynchronizedStart(dir_a, dir_b, speed);
}

/**
 * @brief Configura velocidad �ptima para 4.5V (~38% con fuente 12V)
 */
void Motor_SetOptimalSpeed(motor_id_t motor_id, motor_direction_t direction)
{
    Motor_Control(motor_id, direction, MOTOR_OPTIMAL_VOLTAGE_PCT);
}

/**
 * @brief Configura velocidad para m�ximo torque
 */
void Motor_SetTorqueSpeed(motor_id_t motor_id, motor_direction_t direction)
{
    Motor_Control(motor_id, direction, MOTOR_TORQUE_SPEED);
}

/**
 * @brief Arranque suave para reductores (evita da�os)
 */
void Motor_StartSlow(motor_id_t motor_id, motor_direction_t direction)
{
    Motor_SetDirection(motor_id, direction);
    
    // Arranque gradual desde velocidad m�nima
    for (uint8_t speed = MOTOR_MIN_SPEED_REDUCTION; speed <= MOTOR_OPTIMAL_VOLTAGE_PCT; speed += 3) {
        Motor_SetSpeed(motor_id, speed);
        __delay_ms(100);  // 100ms entre incrementos
    }
}

// =============================================================================
// FUNCIONES ESPEC�FICAS PARA REDUCTORES 180:1
// =============================================================================

// =============================================================================
// FUNCIONES DE ROBOT (CONTROL COORDINADO CON SINCRONIZACI�N)
// =============================================================================

void Robot_MoveForward(uint8_t speed)
{
    Motors_SynchronizedDirection(MOTOR_FORWARD, MOTOR_FORWARD, speed);
}

void Robot_MoveBackward(uint8_t speed)
{
    Motors_SynchronizedDirection(MOTOR_BACKWARD, MOTOR_BACKWARD, speed);
}

void Robot_TurnLeft(uint8_t speed)
{
    // Motor izquierdo m�s lento para giro
    Motor_SetDirection(MOTOR_A, MOTOR_FORWARD);
    Motor_SetDirection(MOTOR_B, MOTOR_FORWARD);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);
    
    Motor_SetSpeed(MOTOR_A, speed/2);
    Motor_SetSpeed(MOTOR_B, speed);
}

void Robot_TurnRight(uint8_t speed)
{
    // Motor derecho m�s lento para giro
    Motor_SetDirection(MOTOR_A, MOTOR_FORWARD);
    Motor_SetDirection(MOTOR_B, MOTOR_FORWARD);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);
    
    Motor_SetSpeed(MOTOR_A, speed);
    Motor_SetSpeed(MOTOR_B, speed/2);
}

void Robot_RotateLeft(uint8_t speed)
{
    Motors_SynchronizedDirection(MOTOR_BACKWARD, MOTOR_FORWARD, speed);
}

void Robot_RotateRight(uint8_t speed)
{
    Motors_SynchronizedDirection(MOTOR_FORWARD, MOTOR_BACKWARD, speed);
}

void Robot_Stop(void)
{
    Motors_SynchronizedStop();
}

// Nuevas funciones con arranque r�pido
void Robot_FastForward(void)
{
    Motors_FastStart_ThenConstant(MOTOR_FORWARD, MOTOR_FORWARD);
}

void Robot_FastBackward(void)
{
    Motors_FastStart_ThenConstant(MOTOR_BACKWARD, MOTOR_BACKWARD);
}

// =============================================================================
// SECUENCIAS DE DEMOSTRACI�N
// =============================================================================

/**
 * @brief Demo 1: Movimientos b�sicos con arranque r�pido sincronizado
 */
void Demo_BasicMovements(void)
{
    // Adelante con arranque r�pido ? velocidad constante
    Robot_FastForward();
    delay_seconds(4);
    
    Robot_Stop();
    delay_half_second();
    
    // Atr�s con arranque r�pido ? velocidad constante
    Robot_FastBackward();
    delay_seconds(4);
    
    Robot_Stop();
    delay_half_second();
    
    // Giros sincronizados
    Robot_RotateLeft(MOTOR_TORQUE_SPEED);
    delay_seconds(2);
    
    Robot_Stop();
    delay_half_second();
    
    Robot_RotateRight(MOTOR_TORQUE_SPEED);
    delay_seconds(2);
    
    Robot_Stop();
}

/**
 * @brief Demo 2: Test de sincronizaci�n perfecta
 */
void Demo_PrecisionControl(void)
{
    // Test de arranque sincronizado con diferentes velocidades
    uint8_t test_speeds[] = {30, 45, 60};
    
    for (uint8_t i = 0; i < 3; i++) {
        // Adelante con arranque r�pido
        Motors_SynchronizedStart(MOTOR_FORWARD, MOTOR_FORWARD, test_speeds[i]);
        delay_seconds(3);
        
        // Parada sincronizada
        Motors_SynchronizedStop();
        delay_seconds(1);
        
        // Atr�s con arranque r�pido
        Motors_SynchronizedStart(MOTOR_BACKWARD, MOTOR_BACKWARD, test_speeds[i]);
        delay_seconds(3);
        
        Motors_SynchronizedStop();
        delay_seconds(1);
    }
}

/**
 * @brief Demo 3: Test de torque y arranque r�pido
 */
void Demo_TorqueTest(void)
{
    // Test de arranque r�pido m�ltiple
    for (uint8_t i = 0; i < 3; i++) {
        Robot_FastForward();
        delay_seconds(2);
        
        Robot_Stop();
        delay_half_second();
        
        Robot_FastBackward();
        delay_seconds(2);
        
        Robot_Stop();
        delay_half_second();
    }
    
    // Test de rotaciones sincronizadas
    Robot_RotateLeft(MOTOR_TORQUE_SPEED);
    delay_seconds(1);
    
    Robot_RotateRight(MOTOR_TORQUE_SPEED);
    delay_seconds(1);
    
    Robot_Stop();
}

/**
 * @brief Convierte porcentaje a valor PWM
 */
uint16_t ConvertPercentToPWM(uint8_t percentage)
{
    if (percentage > 100) percentage = 100;
    return (uint16_t)((percentage * 999UL) / 100);
}

/*
 * =============================================================================
 * NOTAS ESPEC�FICAS PARA MOTORES CON REDUCTOR 180:1:
 * =============================================================================
 * 
 * 1. CARACTER�STICAS DE LOS REDUCTORES:
 *    - Reducci�n 180:1 = ALTO TORQUE, BAJA VELOCIDAD
 *    - Voltaje �ptimo: 4.5V (38% de 12V)
 *    - Material pl�stico: requiere arranque suave
 *    - Velocidad m�nima: 15% para vencer la reducci�n
 * 
 * 2. SOLUCI�N A PROBLEMAS DE SINCRONIZACI�N:
 *    ? Motors_FastStart_ThenConstant() - Arranque r�pido + velocidad constante
 *    ? Motors_SynchronizedStart() - Arranque perfectamente sincronizado
 *    ? Motors_SynchronizedStop() - Parada simult�nea
 *    ? SYNC_DELAY_BETWEEN_MOTORS = 5ms - Elimina desfases
 *    ? STARTUP_SPEED = 70% inicial ? RUNNING_SPEED = 38% constante
 * 
 * 3. SECUENCIA DE ARRANQUE OPTIMIZADA:
 *    FASE 1: Configurar direcciones simult�neamente
 *    FASE 2: Arranque r�pido (70% por 300ms) para vencer inercia
 *    FASE 3: Reducir a velocidad constante (38%) para operaci�n normal
 *    FASE 4: Mantener sincronizaci�n durante toda la operaci�n
 * 
 * 4. NUEVAS FUNCIONES DISPONIBLES:
 *    ? Robot_FastForward() - Adelante con arranque r�pido
 *    ? Robot_FastBackward() - Atr�s con arranque r�pido
 *    ? Motors_SynchronizedDirection() - Control direccional sincronizado
 * 
 * 4. CONEXIONES F�SICAS:
 *    Motor A (Izquierdo):  RC2(PWM), RA0(IN1), RA1(IN2)
 *    Motor B (Derecho):    RC1(PWM), RA2(IN3), RA3(IN4)
 *    Fuente: 12V 2A ? L298N ? Motores 3-6V
 * 
 * 5. APLICACIONES IDEALES:
 *    - Robots con precisi�n de movimiento
 *    - Aplicaciones que requieren alto torque
 *    - Control de posici�n con encoders
 *    - Veh�culos de carga pesada
 * 
 * =============================================================================
 */