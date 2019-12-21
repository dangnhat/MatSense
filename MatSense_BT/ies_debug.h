/**
 * @file ies_debug.h
 * @author  Nhat Pham <nhat.pham@colorado.edu>.
 * @version 1.0
 * @date 21-Sep-2018
 * @brief This contains marcos for debugging and notification.
 */

#ifndef APPLICATION_IES_DEBUG_H_
#define APPLICATION_IES_DEBUG_H_

#include <ti/display/Display.h>

#define IES_STATE       0
#define IES_RESULT1    0
#define IES_RESULT2    0

extern Display_Handle displayHandle;
#define DISPLAY_HANDLE  displayHandle

#if IES_NOTIFICATION_EN
#define IES_NOTIFY(...) Display_printf(__VA_ARGS__)
#else
#define IES_NOTIFY(...)
#endif

#if IES_DEBUG_EN
#define IES_DEBUG(...) Display_printf(__VA_ARGS__)
#define IES_PRINTF(...) ies_printf(__VA_ARGS__)
#else
#define IES_DEBUG(...)
#define IES_PRINTF(...)
#endif

#define IES_DEBUG_STATE(...)    IES_DEBUG(DISPLAY_HANDLE, IES_STATE, 0, (char *) __VA_ARGS__)
#define IES_DEBUG_RESULT1(...)  IES_DEBUG(DISPLAY_HANDLE, IES_RESULT1, 0,(char *) __VA_ARGS__)
#define IES_DEBUG_RESULT2(...)  IES_DEBUG(DISPLAY_HANDLE, IES_RESULT2, 0, (char *) __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

void ies_printf(const char *format, ...);
void ies_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_IES_DEBUG_H_ */
