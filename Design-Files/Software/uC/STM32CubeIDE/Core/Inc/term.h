/**
  ******************************************************************************
  * @file    term.h
  * @author  Dominik Muth          
  * @brief   Configuration and Usage of the terminal Program         
	* @date    2020-08-27
  ******************************************************************************
**/
#ifndef __TERM_H 
#define __TERM_H 

/* Includes ------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

typedef struct intern_var_t {
	unsigned char nr;
  volatile long unsigned int* value;
} intern_var_t;

extern intern_var_t sInternVar[];

/* Variables -----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

void
	CycleTerm
		(void);

#endif /* __TERM_H */
/* END ************************************************************************/
