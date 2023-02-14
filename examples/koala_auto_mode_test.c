#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <koala/koala.h> // koala library

// Check that koala's functions work correctly
// Parametrs:
// 1) text - error message
// 2) error - koala function result
// 3) error_return - error id(is not neccesary). By default error_return = -1
#define IF_ERROR(text, error, error_return = -1) \
	{                                            \
		if (error < 0)                           \
		{                                        \
			fprintf(stderr, text, error);        \
			return error_return;                 \
		}                                        \
	}

void PrintInit(){
	printf("Koala V2.5 robot Auto mode test program\n(C) K-Team S.A\n");
}
void PrintVersion(char *version, char *revision) {
	printf("Koala version: %c  revision: %d\n", *version, *revision);
}
void PrintMotorSpeed(const koala_auto_data_t *data)
{
	printf("\nMotor speed  : left: %5d  right: %5d\n", data->left_speed, data->right_speed);
}
void PrintUSsensor(const koala_auto_data_t *data)
{
	printf("\nUS sensors:\n");
	for (int i = 0; i < KOALA_US_SENSORS_NUMBER; i++)
	{
		printf("  US %d: %d\n", i, data->us[i]);
	}
}
void PrintAccelerometerValue(const koala_auto_data_t *data)
{
	int dmean = 0, dval = 0, i = 0;
	printf("\nAcceleration sensor [g]\n       new data                                                      old data    average\naxis X: ");

	for (i = 0; i < KOALA_ACCEL_VALUES_NUMBER; i += 3)
	{
		dval = data->accel[i] * KOALA_ACCEL_G;
		printf("%6.1f ", dval);
		dmean += dval;
	}

	printf(" %6.1f", dmean / 10.0);

	printf("\naxis Y: ");
	dmean = 0;
	for (i = 1; i < KOALA_ACCEL_VALUES_NUMBER; i += 3)
	{

		dval = data->accel[i] * KOALA_ACCEL_G;
		printf("%6.1f ", dval);
		dmean += dval;
	}
	printf(" %6.1f", dmean / 10.0);

	printf("\naxis Z: ");
	dmean = 0;
	for (i = 2; i < KOALA_ACCEL_VALUES_NUMBER; i += 3)
	{
		dval = data->accel[i] * KOALA_ACCEL_G;
		printf("%6.1f ", dval);
		dmean += dval;
	}
	printf(" %6.1f", dmean / 10.0);
	printf("\n");
}
void PrintGyroscopeValue(const koala_auto_data_t *data)
{
	printf("\ngyro sensor [deg/s]\n       new data                                                      old data    average\ngyro X: ");
	int dmean = 0, dval = 0, i = 0;
	for (i = 0; i < KOALA_GYRO_VALUES_NUMBER; i += 3)
	{
		dval = data->gyro[i] * KOALA_GYRO_DEG_S;
		printf("%6.1f ", dval);
		dmean += dval;
	}

	printf(" %6.1f", dmean / 10.0);

	printf("\ngyro Y: ");
	dmean = 0;
	for (i = 1; i < KOALA_GYRO_VALUES_NUMBER; i += 3)
	{

		dval = data->gyro[i] * KOALA_GYRO_DEG_S;
		printf("%6.1f ", dval);
		dmean += dval;
	}
	printf(" %6.1f", dmean / 10.0);

	printf("\ngyro Z: ");
	dmean = 0;
	for (i = 2; i < KOALA_GYRO_VALUES_NUMBER; i += 3)
	{
		dval = data->gyro[i] * KOALA_GYRO_DEG_S;
		printf("%6.1f ", dval);
		dmean += dval;
	}
	printf(" %6.1f", dmean / 10.0);
	printf("\n");
}
void PrintMotorCurrent(const koala_auto_data_t *data) {
	printf("\nMotor current: left: %5d  right: %5d\n", data->left_current, data->right_current);
}
void PrintMotorPosition(const koala_auto_data_t *data) {
	printf("\nMotor current: left: %5d  right: %5d\n", data->left_position, data->right_position);
}
void PrintGPSdata(const koala_auto_data_t *data) {
	printf("\nGPS data:\
					 				\n  valid mode: %c\
					        \n  sat number: %2d\
					        \n  latitude  : %5.4f %c\
					        \n  longitude : %5.4f %c\
					        \n  time      : %02d:%02d:%02d\
					        \n  date      : %02d.%02d.%02d\
					        \n  speed     : %3.1f [knots]\
					        \n  altitude  : %5d [m]\n",
		   data->gps.valid_sat, data->gps.sat_nb, data->gps.lat_val, data->gps.lat_car, data->gps.long_val, data->gps.long_car, data->gps.date_time.tm_hour, data->gps.date_time.tm_min, data->gps.date_time.tm_sec, data->gps.date_time.tm_mday, data->gps.date_time.tm_mon, data->gps.date_time.tm_year, data->gps.speed, data->gps.altitude);
}
void PrintGPSraw(const koala_auto_data_t *data) {
	printf("\nGPS raw data: %s\n", data->gps_raw);
}
void PrintMagnetometerData(const koala_auto_data_t *data) {
	printf("\nMagnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n", data->magne[0], data->magne[1], data->magne[2]);
}

/*!
 * Main program
 *
 * \param argc number of arguments
 * \param argv argument array
 *
 * \return 0 if OK, <0 if error
 *
 * For example C:/from/to/main.exe argument1.txt argument2.json
 * It means that argc=2 and argv={"argument1.txt", "argument2.json"}
 */
int main(int argc, char *argv[])
{
	// Helpers
	int error = 0;		   // check error if rc < 0 ---> error
	//int i;				   // iterator(is used in cycle for)
	unsigned int bit_conf; // bit configuration for koala_configure_auto_monitoring_mode

	// Accelerometer and gyroskope parametrs
	//double dval, dmean;

	// Version parapetrs
	char revision, version;

	// Receiving automatic data from the world
	koala_auto_data_t data;

	// initialise koala library
	error = koala_init(argc, argv);
	IF_ERROR("ERROR %d: Unable to initialize the koala library!\n", error, -1);
	PrintInit();

	// get robot firmware version and revision
	error = koala_get_firmware_version_revision(&version, &revision);
	if (error < 0) // retry, because of the serial start level
	{
		error = koala_get_firmware_version_revision(&version, &revision);
		IF_ERROR("ERROR %d: Koala did not respond correctly!\n", error, -2);
	}
	PrintVersion(version, revision);
	

	// Set auto mode configuration
	bit_conf = KOALA_AUTOM_MOTOR_POSITION | KOALA_AUTOM_MAGNE_VALUE;
	error = koala_configure_auto_monitoring_mode(bit_conf);
	IF_ERROR("ERROR %d: could not set auto mode configuration!\n", error, -4);

	// Waiting command every 10 seconds
	printf("Read serial until any key pushed!\n\n");
	while (koala_kbhit() == 0) // wait buuton pushing ...
	{
		// if automatic contains something ...
		if (koala_get_from_auto_mode(&data) >= 0)
		{
			// commands: e, g, m, n, o, p, q, $, @
			switch (data.mode)
			{
			case 'e': // Motor Speed
				PrintMotorSpeed(&data);
				break;
			case 'g': // US sensor
				PrintUSsensor(&data);
				break;
			case 'm': // Accelerometer value
				PrintAccelerometerValue(&data);
				break;
			case 'n': // Gyroscope value
				PrintGyroscopeValue(&data);
				break;
			case 'o': // Motor Current
				PrintMotorCurrent(&data);
				break;
			case 'p': // Motor Position
				PrintMotorPosition(&data);
				break;
			case 'q': // GPS data
				PrintGPSdata(&data);
				break;
			case '$': // GPS raw data
				PrintGPSraw(&data);
				break;
			case '@': // Magnetometer data
				PrintMagnetometerData(&data);
				break;
			default: // Unknown command
				printf("\nERROR: received invalid auto mode: %c (0x%x)\n", data.mode, data.mode);
			}
		}

		// wait 10 seconds
		usleep(10000);
	}

	// exit auto configuration
	bit_conf = KOALA_AUTOM_NONE;
	koala_configure_auto_monitoring_mode(bit_conf);

	return 0;
}
