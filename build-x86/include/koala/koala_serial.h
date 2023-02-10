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


extern koala_rs232_t * koala_rs232_open(const char * name, int baudrate);
					    
extern void koala_rs232_close(koala_rs232_t * rs232 );

extern int koala_rs232_read(koala_rs232_t * rs232, char * buf , unsigned int len );
			   
			   
extern int koala_rs232_readLine_nowait(koala_rs232_t * rs232, char *buffer);
extern int koala_rs232_readLine(koala_rs232_t * rs232, char *buffer);
			    
extern int koala_rs232_write(koala_rs232_t * rs232 ,
			   const char * buf , unsigned int len );			
			   
			       

#endif /* __koala_serial__ */
