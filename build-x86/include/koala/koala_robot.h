#ifndef __koala_robot__
#define __koala_robot__

#include <time.h>
#include <math.h>

/*!
 *  Koala Constants
 */

#define KOALA_SERIAL_PORT_NAME "/dev/ttyS1"
#define KOALA_SERIAL_PORT_BAUDRATE B115200

#define KOALA_MAX_BUFFER 1024 // max input/output buffer for sending/receiving data

#define KOALA_DELIM ","  // delimiter for parameters

// bit configuration for auto monitoring mode
#define KOALA_AUTOM_US_SENSOR_BIT  1<<0	// US sensor
#define KOALA_AUTOM_MOTOR_SPEED    1<<1	// Motor Speed
#define KOALA_AUTOM_MOTOR_POSITION 1<<2	// Motor Position
#define KOALA_AUTOM_MOTOR_CURRENT  1<<3	// Motor Current
#define KOALA_AUTOM_ACCEL_VALUE    1<<4	// Accelerometer value
#define KOALA_AUTOM_GYRO_VALUE     1<<5	// Gyroscope value
#define KOALA_AUTOM_GPS_DATA       1<<6	// GPS data
#define KOALA_AUTOM_GPS_NEMA       1<<7	// GPS NEMA Data (will return all GPS raw data)
#define KOALA_AUTOM_MAGNE_VALUE    1<<8	// Magnometer value
#define KOALA_AUTOM_ALL            0x01FF  // all options activated
#define KOALA_AUTOM_NONE           0x0000  // all options desactivated


// bit configuration for US sensors mask
#define KOALA_US_LEFT_REAR	 1<<0  // Left rear
#define KOALA_US_LEFT_FRONT	 1<<1  // Left front
#define KOALA_US_FRONT_LEFT	 1<<2  // Front left
#define KOALA_US_FRONT       1<<3  // Front
#define KOALA_US_FRONT_RIGHT 1<<4  // Front left 
#define KOALA_US_RIGHT_FRONT 1<<5  // Right front
#define KOALA_US_RIGHT_REAR  1<<6  // Right rear
#define KOALA_US_BACK_RIGHT  1<<7  // Back right
#define KOALA_US_BACK_LEFT   1<<8  // Back left
#define KOALA_US_ALL         511   // all US activated
#define KOALA_US_NONE        0     // all US desactivated

extern const char  *KOALA_US_SENSOR_NAMES[];


#define KOALA_US_SENSORS_NUMBER 9 // number of US sensors

// bit configuration for Input/Output ports mask
#define KOALA_IO_0_OUTPUT   0b00000000  // Port 0 in output
#define KOALA_IO_0_INPUT    0b00000001  // Port 0 in input
#define KOALA_IO_0_PWMS     0b00000010  // Port 0 in PWM servo
#define KOALA_IO_1_OUTPUT   0b00000000  // Port 1 in output
#define KOALA_IO_1_INPUT    0b00000100  // Port 1 in input
#define KOALA_IO_1_PWMS     0b00001000  // Port 1 in PWM servo
#define KOALA_IO_2_OUTPUT   0b00000000  // Port 2 in output
#define KOALA_IO_2_INPUT    0b00010000  // Port 2 in input
#define KOALA_IO_2_PWMS     0b00100000  // Port 2 in PWM servo
#define KOALA_IO_3_OUTPUT   0b00000000  // Port 2 in output
#define KOALA_IO_3_INPUT    0b01000000  // Port 2 in input
#define KOALA_IO_3_PWMS     0b10000000  // Port 2 in PWM servo
#define KOALA_IO_ALL_OUTPUT 0b00000000  // All IO ports in output


// bit configuration for POWER Input/Output ports mask
#define KOALA_PWR_IO_0_0    0b00000000  // Power Port 0 to 0
#define KOALA_PWR_IO_0_1    0b00000001  // Power Port 0 to 1
#define KOALA_PWR_IO_1_0    0b00000000  // Power Port 1 to 0
#define KOALA_PWR_IO_1_1    0b00000010  // Power Port 1 to 1
#define KOALA_PWR_IO_2_0    0b00000000  // Power Port 2 to 0
#define KOALA_PWR_IO_2_1    0b00000100  // Power Port 2 to 1
#define KOALA_PWR_IO_3_0    0b00000000  // Power Port 3 to 0
#define KOALA_PWR_IO_3_1    0b00001000  // Power Port 3 to 1



#define KOALA_ACCEL_G (1.0/16384.0)   // convert to [g]
#define KOALA_ACCEL_VALUES_NUMBER 30 // number of values from the accelerometer: 3 axes * 10 values

#define KOALA_NEW_ACCEL_X  0   // position of newest X acceleration in the buffer
#define KOALA_NEW_ACCEL_Y  1  // position of newest Y acceleration in the buffer
#define KOALA_NEW_ACCEL_Z  2  // position of newest Z acceleration in the buffer

#define KOALA_GYRO_DEG_S (66.0/1000.0) // convert to [deg/s]
#define KOALA_GYRO_VALUES_NUMBER 30  // number of values from the gyrometer: 3 axes * 10 values
#define KOALA_NEW_GYRO_X  0    // position of newest X speed in the buffer
#define KOALA_NEW_GYRO_Y  1   // position of newest Y speed in the buffer
#define KOALA_NEW_GYRO_Z  2   // position of newest Z speed in the buffer

#define KOALA_MAGNE_VALUES_NUMBER 3  // x, y ,z

#define KOALA_MAX_I2C_DATA 256 // max i2c data to be send/received at one time


// robot hardware constants

#define KOALA_WHEELS_DISTANCE 250.0 // distance between wheel, for rotation calculus [mm]

#define KOALA_WHEEL_DIAM 82.5  // wheel diameter [mm] (real 85; without load 82.5, with 3kg load: 80.5)

#define KOALA_PULSE_TO_MM  (M_PI*KOALA_WHEEL_DIAM/23400.0) // motor position factor to convert from pulse to mm

#define KOALA_TIME_BTWN 10 // [ms] time for speed computation

#define KOALA_SPEED_TO_MM_S  (KOALA_PULSE_TO_MM/(KOALA_TIME_BTWN/1000.0)) // motor speed factor to convert from speed units to mm/s 

#define KOALA_US_DISABLED_SENSOR 2000  // disabled sensor
#define KOALA_US_NO_OBJECT_IN_RANGE 1000  // no object in range 25..250cm
#define KOALA_US_OBJECT_NEAR	0 // object at less 25cm

// default motor parameters
#define KOALA_MOTOR_P 10
#define KOALA_MOTOR_I  3
#define KOALA_MOTOR_D  1
#define KOALA_MOTOR_ACC_INC 5
#define KOALA_MOTOR_ACC_DIV 1
#define KOALA_MOTOR_MIN_SPACC 10
#define KOALA_MOTOR_CST_SPEED 200
#define KOALA_MOTOR_POS_MARGIN 10
#define KOALA_MOTOR_MAX_CURRENT 10 


#define KOALA_AD_TO_V (3.3/1024) // convert AD valu to Volt

/*!
 *  Koala Error codes
 */

#define KOALA_RS232_MESSAGE_ERROR_CHAR '#'

/*!

 *  Koala types
 */
 
// koala struct type for gps data
typedef struct  gps_data_s
{ 
	char valid_sat;			// valid data flag	
	int sat_nb;         // number of satellites used
	double lat_val;     // latitude
	char lat_car;       // latitude N or S
	double long_val;    // longitude
	char long_car; 			 // longitude W or E
	struct tm date_time; // UTC date and time of the last fix
	double	speed;		   // speed in knots
	int altitude;	       // altitude in meter
} gps_data_t;
 
 // koala struct type for auto mode
typedef struct auto_struct_s
{ 
  // for speed, current, position 
	int left_speed;     // motor left speed
	int right_speed;    // motor right speed
	int left_position;  // motor left position
	int right_position; // motor right position
	int left_current;   // motor left current
	int right_current;  // motor right current
	int us[KOALA_US_SENSORS_NUMBER]; // us sensors
	int accel[KOALA_ACCEL_VALUES_NUMBER]; // accelerometer
	int gyro[KOALA_GYRO_VALUES_NUMBER];   // gyrometer
	int magne[KOALA_MAGNE_VALUES_NUMBER]; // magnometer
	char gps_raw[KOALA_MAX_BUFFER];
	char mode; // type of the data received
	gps_data_t gps;
} koala_auto_data_t;


/*!--------------------------------------------------------------------
 * Prototypes Declaration
 */
 

// Is called by koala_init

/*
****Прототипы объявления инициализируются.
	Эта функция должна быть вызвана перед любыми другими функциями.
	Но по умолчанию это уже называется внутри koala_init!
****Prototypes Declaration initializes. 
	This function needs to be called BEFORE any other functions. 
	But default it is already called inside koala_init!*/
extern int koala_robot_init( void );
/*
Выпустить робота.
Эта функция должна быть вызвана после любых других функций.
Release the robot. 
This function needs to be called AFTER any other functions.*/
extern int koala_robot_release( void );


// "Low level" function to communicate with the KOALA via serial

/*
***Отправьте командный кадр из робота.
   Обычно конечный пользователь не хочет использовать эту функцию, поскольку они считаются «функциями низкого уровня».
***Send a command frame from the robot.
   Normally an end user don't want to use these function as they are assumed as "low level functions".

Parameters:
	***send_len - is the size of the message received
	***command - is a pointer to a buffer where the command frame to send*/
extern int koala_sendcommand(char *command,int write_len);
/*
Получает командный кадр от робота.
gets a command frame from the robot.
Normally an end user don't want to use these function as they are assumed as "low level functions".

Parameters:
	***read_len	(is the size of the message received)
	***command	(is a pointer to a buffer where the command frame will be stored in.)
*/
extern int koala_getcommand(char *command,int read_len);
/*
Получает командную строку от робота.
gets a command line from the robot.
Normally an end user don't want to use these function as they are assumed as "low level functions".

Parameters:
	***command (is a pointer to a buffer where the command frame will be stored in.)
*/
extern int koala_getcommand_line(char *command);

// "High level" function that let user to simply retrieve various informations from the robot

/*
***Извлекает текущую версию прошивки OS Firmware version/revision
***retrieves the current OS Firmware version/revision

Parameters:
	***version	- version
	***revision	- revision*/
extern int koala_get_firmware_version_revision(char *version,char *revision);
/*
Прочитайте различные значения батареи
Read the different values of the battery

Parameters:
	***bat_type	
		0 = Li-ION, 
		1 = NiMH, 
		2 = not initialised
	***bat_voltage	
		Voltage of the battery (unit is 0.1V)
	***bat_current	
		Current of the battery (unit is 0.1A)
	***chrg_current	
		Charge Current (unit is 10mA)*/
extern int koala_get_battery_data(int *bat_type,int *bat_voltage, int *bat_current,int *chrg_current);



/*
*** Включить периферийное устройство автоматически, не спрашивая его значение при обновлении
*** Enable a peripheral to return automatically without asking its value when refreshed

Parameters:
bit_config	binary OR bit mask configuration for auto monitoring mode (default: all OFF) see koala_robot.h for constant definition of each 
Bit 0: US sensor 
Bit 1: Motor Speed 
Bit 2: Motor Position 
Bit 3: Motor Current 
Bit 4: Accelerometer value 
Bit 5: Gyroscope value 
Bit 6: GPS data 
Bit 7: GPS NEMA Data (will return all GPS raw data) 
Bit 8: Magnometer value*/
extern int koala_configure_auto_monitoring_mode(unsigned int bit_config);
/*
***Получить значения из автозаправления
***Get values from auto mode

Parameters:
	***data - data received

koala_auto_data_t: (for speed, current, position) 
	int left_speed;     // motor left speed
	int right_speed;    // motor right speed
	int left_position;  // motor left position
	int right_position; // motor right position
	int left_current;   // motor left current
	int right_current;  // motor right current
	int us[KOALA_US_SENSORS_NUMBER]; // us sensors
	int accel[KOALA_ACCEL_VALUES_NUMBER]; // accelerometer
	int gyro[KOALA_GYRO_VALUES_NUMBER];   // gyrometer
	int magne[KOALA_MAGNE_VALUES_NUMBER]; // magnometer
	char gps_raw[KOALA_MAX_BUFFER];
	char mode; // type of the data received
	gps_data_t gps;
*/
extern int koala_get_from_auto_mode(koala_auto_data_t *data);
/*
***Настройте режим автоматического мониторинга.Включите периферийное устройство автоматически, не спрашивая его значение при обновлении.
***Configure the auto monitoring mode. Enable a peripheral to return automatically without asking its value when refreshed.

Parameters:
***us_mask(binary OR bit mask configuration for use of Ultrasonic sensors (default: all active) see koala_robot.h for constant definition of each)
	Bit 0: Left rear 
	Bit 1: Left front 
	Bit 2: Front left 
	Bit 3: Front 
	Bit 4: Front right 
	Bit 5: Right front 
	Bit 6: Right rear 
	Bit 7: Back right 
	Bit 8: Back left
***io_dir(binary OR bit mask configuration for direction of the four IO (0..3)). 
	Each IO is configure with two following bits: (IO0 = bit0 & 1, IO1 = bit2&3,…). 
	Default: 
		0 = all output 
		00 = output 
		01 = input 
		10 = PWM servo (50Hz). 
	see koala_robot.h for constant definition of each*/
extern int koala_configure_us_io(int us_mask,unsigned char io_dir);



/*
Настройте значение контроллера PID, используемое для управления скоростью.
Configure the PID controller value used for the speed control.

Parameters:
kp - P parameter - Get Encoder value (Читает фактическое значение кодера позиции)
ki - I parameter - Set the position encoder value (Сбрасывает значение кодера положения двигателя)
kd - D parameter - Set Motor speed (Устанавливает скорость двигателя)*/
extern int koala_configure_pid(int kp, int ki,int kd);
/*
***Настройте параметры, используемые для управления позицией.
***Configure the parameters used for the position control.

Parameters:
	***acc_inc - Increment of the speed added every “Acc_Div+1” control loop.(Приращение скорости добавлена каждый цикл управления «ACC_DIV+1».)
	***acc_div - Number of control loop before adding the Acc_Inc to the speed order(Количество цикла управления перед добавлением ACC_INC в заказ скорости)
	***min_speed - Minimum speed order used during the position control (Минимальный заказ скорости, используемый во время управления положением)
	***cst_speed - Constant speed used during the position control after acceleration (Постоянная скорость, используемая во время управления положением после ускорения)
	***pos_margin - Margin of the position control to detect when the robot reach its target. (Край контроля положения, чтобы обнаружить, когда робот достигает своей цели.)
	***max_current - Maximum current for each motor. If above this value, the controller will limit the motor command. (unit is mA, 0 = disable (default)). (Максимальный ток для каждого двигателя.Если выше этого значения контроллер ограничит команду двигателя.(единица - это ma, 0 = отключить (по умолчанию)))*/
extern int koala_set_speed_profile(int acc_inc,int acc_div,int min_speed,int cst_speed,int pos_margin,int max_current);
/*
***Установите значение кодера положения [импульсы].
   Сбросить значение кодера положения двигателя.
   Если установлено во время движения управления позицией, двигатели будут остановлены, чтобы избежать неправильного поведения.
***Set the position encoder value [pulses].
   Reset the position encoder value of the motor. 
   If set during a position control move, the motors will be stopped to avoid an incorrect behaviour.

Parameters:
	***left - left encoder position
	***right - right encoder position*/
extern int koala_set_position_encoders(int left, int right);



/*
***Установите скорость двигателя [импульсы / koala_time_btwn MS].
   Контроллер PID будет управлять скоростью в закрытом цикле.
***Set the motor speed [pulses / KOALA_TIME_BTWN ms]. 
   The PID controller will manage the speed in closed loop.

Parameters:
	***left - left speed
	***right - right speed*/
extern int koala_set_motor_speed(int left, int right);
/*
***Установите скорость двигателя с помощью рампы ускорения [импульсы / koala_time_btwn MS].
Установите заказ скорости, чтобы достичь с помощью рампы ускорения.
Параметры, используемые для этого режима, такие же, как и управление позицией.
***Set Motor Speed with acceleration ramp [pulses / KOALA_TIME_BTWN ms].
Set a speed order to reach with acceleration ramp. 
The parameters used for this mode are the same as the position control.

Parameters:
	***left - left speed
	***right - right speed*/
extern int koala_set_motor_speed_accel(int left, int right);
/*
***Установите скорость в управлении открытым циклом.
Установите значение PWM для каждого двигателя.
Значение может быть от -2000 до +2000 (соответствует от -100 до 100% PWM).
***Set speed in open loop control.
Set a PWM value for each motor. 
Value can be from -2000 to +2000 (corresponding to -100 to 100% of PWM).

Parameters:
	***left - left pwm
	***right - right pwm*/
extern int koala_set_motor_speed_open_loop(int left, int right);
/*
***Прочитайте фактическую скорость двигателя [импульсы / koala_time_btwn MS].
***Read the motor actual speed [pulses / KOALA_TIME_BTWN ms].

Parameters:
	***left - left speed
	***right - right speed*/
extern int koala_read_motor_speed(int *left, int *right);
/*
***Установите положение целевого двигателя [pulses].
***Set the motor target position [pulses].

Parameters:
	***left - left target position
	***right - right target position*/
extern int koala_set_motor_target_position(int left, int right);
/*
***Прочитайте ток двигателя
***Read the motor current

Parameters:
	***left - current of left motor [0.1 A]
	***right - current of right motor [0.1 A]*/
extern int koala_read_motor_current(int *left, int *right);
/*
***Прочитайте положение двигателя [Pulse]
***Read the motor position [pulse]

Parameters:
	***left - encoder value of left motor
	***right - encoder value of right motor*/
extern int koala_read_motor_position(int *left, int *right);
/*
***Get Motor control status
***Read the actual status of the motor control
***Получите статус управления двигателем
***Прочитайте фактическое состояние управления двигателем

Parameters:
	***left_status(left motor control status Type of actual control): 
		0: Idle 
		1: Speed 
		2: Speed with Acceleration 
		3: Position 
		4: Open Loop 
		5: Current Limitation 
		6: Error
	***right_status(right motor control status):
		0: Idle 
		1: Speed 
		2: Speed with Acceleration 
		3: Position 
		4: Open Loop 
		5: Current Limitation 
		6: Error
	***left_pos(left motor): 
		1 if the target is reach(Если цель достигается) (position control)
	***right_pos(right motor): 
		1 if the target is reach(Если цель достигается) (position control)*/
int koala_get_motor_status(int *left_status, int *right_status,int *left_pos, int *right_pos);



/*
Прочитайте ультразвуковые датчики
Read the ultrasonic sensors

Parameters:
	***values_array(sensors values array)
		0: Obstacle <25cm 
		25-250: Obstacle distance in cm 
		1000: No obstacle detected 
		2000: Sensor disable*/
extern int koala_read_us_sensors(int *values_array);
/*
***Получите значение акселерометра: прочитайте последние 10 значений ускорения XYZ.
***Get Accelerometer value: read the last 10 value of the XYZ acceleration.

Parameters:
	***values_array	- in order x0,y0,z0,x1,y1,z1,...,x9,y9,z9; new values first*/
extern int koala_read_accelerometer(int *values_array);
/*
***Получите значение гироскопа: прочитайте последние 10 значений XYZ.
***Get Gyroscope value: read the last 10 value of the XYZ.

Parameters:
	****values_array - in order x0,y0,z0,x1,y1,z1,...,x9,y9,z9; new values first
*/
extern int koala_read_gyroscope(int *values_array);
/*
***Прочтите магнометр
***Read the magnometer

Parameters:
	***values_array - sensors values x,y,z in [mGauss]*/
extern int koala_read_magnetometer(int *values_array);



/*
***Получить данные GPS
***Get GPS data

Parameters:
	***valid_sat - Valid data flag (V = Warning, A = Valid)
	***sat_nb - number of satellites used (количество используемых спутников)
	***lat_val - latitude (широта)
	***lat_car - latitude N or S
	***long_val - longitude (долготу)
	***long_car - longitude W or E
	***date_time - UTC time and time of the latest fix
	***speed - Speed over ground (in knots)
	***altitude - Actual altitude (высота)(in meters)*/
extern int koala_gps_data(char *valid_sat, int *sat_nb,double *lat_val,char *lat_car,double *long_val,char *long_car,struct tm *date_time,double *speed,int *altitude);
/*
***Отправить команду GPS
***Send GPS command

Parameters:
	***gps_cmd - gps command*/
extern int koala_send_gps_cmd(char *gps_cmd);



/*
***Читать внешнюю шину i2c
***Read I2C external bus

Parameters:
	***i2c_add - i2c address
	***i2c_reg - i2c register
	***nb_read - number of bytes to read (max 128)
	***data - bytes read*/
extern int koala_read_i2c(int i2c_add,int i2c_reg, int nb_read,int *data);
/*
***Напишите на внешней шине I2C
***Write on the I2C external bus

Parameters:
	***i2c_add - i2c address
	***i2c_reg - i2c register
	***nb_data - number of data to send
	***data - bytes to be written*/
extern int koala_write_i2c(int i2c_add,int i2c_reg,int nb_data,int *data);
extern int koala_scan_i2c(int *nb_devices,int *address);



/*
***Установите выходное значение PWR и IO Установите состояние четырех вывода PWR 
   и четырех IO (если установлено в качестве вывода, 0 = GND, 1 = +3,3 В, 
   если установлен как Servo, определите положение сервопривода от 0 до 250)
***Set the PWR and IO output value Set the state of the four PWR output 
   and four IO (if set as output, 0 = GND, 1 = +3.3V, 
   if set as PWM servo, define the position of the servo for 0 to 250 ).

Parameters:
	***power_out - power output binary OR bit mask configuration see koala_robot.h for constant definition of each bit
	***IO0 - IO 0
	***IO1 - IO 1
	***IO2 - IO 2
	***IO3 - IO 4*/
extern int koala_set_pwr_io_output(int power_out,int IO0, int IO1, int IO2, int IO3);
/*
***Читайте состояние IO Читать четыре io и два входных состояния.Если IO определяется как вывод, будет то же значение, что и установлено с командой S, если определить как Servo Servo, будет возвращать как 0.
***Read IO state Read the four IO and the two Input state. If the IO are define as output, will be the same value as set with S command, if define as PWM servo output, will be return as 0.

Parameters:
	***io_state - 4 IO in binary OR bit configuration (lsb=0, msb = 3)
	***in_state - 2 digital input*/
extern int koala_read_io(int *io_state, int *in_state);
/*
***Прочитайте два значения ввода AD (0-1024 => 0 - 3,3 В).
***Read the two AD input values (0-1024 => 0 - 3.3V).

Parameters:
	***ad_0 - analog input port 0
	***ad_1 - analog input port 1*/
extern int koala_read_ad(int *ad_0, int *ad_1);
/*
***Сбросить микроконтроллер
***Reset microcontroller*/
extern int koala_reset_microcontroller();

#endif /* __koala_robot__ */
