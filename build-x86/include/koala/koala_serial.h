#ifndef __koala_serial__
#define __koala_serial__

#include <termio.h>

typedef struct koala_rs232_s {
  
  /*! RS-232 device file descriptor */
  int fd;

  /*! */
  struct termios tios;

}
koala_rs232_t;

/*
***Эта функция открывает заданное устройство на шине RS-232
***This function opens a given device on the RS-232 Bus

Parameters:
  ***name - RS-232 device name
  ***baudrate - baudrate (B115200,...)

Returns:
  A Pointer to a RS-232 device descriptor or NULL in case of error.

Remarks:
  This function is NOT exported outside this module.*/
extern koala_rs232_t * koala_rs232_open(const char * name, int baudrate);


/*
***Эта функция закрывает устройство RS-232.
***This function closes an RS-232 device.

Parameters:
  ***rs232 - RS-232 device descriptor*/
extern void koala_rs232_close(koala_rs232_t * rs232 );
/*
***Эта функция считывает данные с устройства RS-232.
***This function reads data from an RS-232 device.

Parameters:
  ***rs232 - RS-232 device descriptor
  ***buf - Pointer to the buffer that will receive the data
  ***len - Size of the buffer

Returns:
  ***<0 - on error
  ***>=0 - on success, number of bytes read*/
extern int koala_rs232_read(koala_rs232_t * rs232, char * buf , unsigned int len );
			   

extern int koala_rs232_readLine_nowait(koala_rs232_t * rs232, char *buffer);
extern int koala_rs232_readLine(koala_rs232_t * rs232, char *buffer);


/*
***Эта функция записывает данные на устройство RS-232.
***This function writes data to an RS-232 device.

Parameters:
  ***rs232 - RS-232 device descriptor
  ***buf - Pointer to the buffer that contains the data to be written
  ***len - Number of the bytes in the buffer

Returns:
  ***<0 - on error
  ***>=0 - on success, number of byte written*/			    
extern int koala_rs232_write(koala_rs232_t * rs232, const char * buf , unsigned int len );					       

#endif /* __koala_serial__ */
