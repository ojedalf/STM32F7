/*-----------------------------------------------------------------------------
   LCD-TFT Display Controller (LTDC)
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Jan-2020
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   List of error parameters for the LTDC driver
-------------------------------------------------------------------------------*/

#ifndef ERRORS_H
#define ERRORS_H

/*--------------------------------------------------------
  Include Files
 *------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>                


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/*--------------------------------------------------------
  Generic
*------------------------------------------------------*/
#define TRUE 1
#define FALSE 0

#define OK 0
#define ERROR -1


/*--------------------------------------------------------
  LCD-TFT Display Controller
*------------------------------------------------------*/
#define LTDC_ERROR_SYNC 457
#define LTDC_ERROR_BUFF 458
#define LTDC_ERROR_AREA 459

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ERRORS_H */
