/* vim: set ai et ts=4 sw=4: */
#ifndef __FONTS_H__
#define __FONTS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" 
{
#endif

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;


extern FontDef Font_16x26;
extern const uint8_t arducam_logo[25608];

#ifdef __cplusplus
}
#endif

#endif // __FONTS_H__
