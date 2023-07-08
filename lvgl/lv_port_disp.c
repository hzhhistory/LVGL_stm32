/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>
#include"lcd.h"

#include"stm32l4xx_it.h"
/*********************
 *      DEFINES
 *********************/
#ifndef MY_DISP_HOR_RES
   // #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen width, default value 320 is used for now.
    #define MY_DISP_HOR_RES    320
#endif

#ifndef MY_DISP_VER_RES
//    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen height, default value 240 is used for now.
    #define MY_DISP_VER_RES    240
#endif

/**********************
 *      TYPEDEFS
 **********************/
extern DMA_HandleTypeDef hdma_memtomem_dma2_channel7;
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_t * disp, const lv_area_t * area, lv_color_t * color_p);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
void lcd_draw_fast_rgb_color(int16_t sx, int16_t sy,int16_t ex, int16_t ey, uint16_t *color)
{
    uint16_t w = ex-sx+1;
    uint16_t h = ey-sy+1;

    LCD_Address_Set(sx, sy, w, h);
    uint32_t draw_size = w * h;


    for(uint32_t i = 0; i < draw_size; i++)
    {
        LCD_Write_HalfWord(color[i]);
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
static lv_disp_t disp_drv1;
void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    lv_disp_t * disp = lv_disp_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    lv_disp_set_flush_cb(disp, disp_flush);

    /* Example 1
     * One buffer for partial rendering*/
    static lv_color_t buf_1_1[MY_DISP_HOR_RES * 10];                          /*A buffer for 10 rows*/
    lv_disp_set_draw_buffers(disp, buf_1_1, NULL, sizeof(buf_1_1), LV_DISP_RENDER_MODE_PARTIAL);

//    /* Example 2
//     * Two buffers for partial rendering
//     * In flush_cb DMA or similar hardware should be used to update the display in the background.*/
//    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];
//    static lv_color_t buf_2_2[MY_DISP_HOR_RES * 10];
//    lv_disp_set_draw_buffers(disp, buf_2_1, buf_2_2, sizeof(buf_2_1), LV_DISP_RENDER_MODE_PARTIAL);
//
//    /* Example 3
//     * Two buffers screen sized buffer for double buffering.
//     * Both LV_DISP_RENDER_MODE_DIRECT and LV_DISP_RENDER_MODE_FULL works, see their comments*/
//    static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];
//    static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];
//    lv_disp_set_draw_buffers(disp, buf_3_1, buf_3_2, sizeof(buf_3_1), LV_DISP_RENDER_MODE_DIRECT);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
    LCD_Init();
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
//    if(disp_flush_enabled) {
//        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
//
//        int32_t x;
//        int32_t y;
//        for(y = area->y1; y <= area->y2; y++) {
//            for(x = area->x1; x <= area->x2; x++) {
//                /*Put a pixel to the display. For example:*/
//               LCD_Draw_ColorPoint(x,y,color_p->green);
//                /*put_px(x, y, *color_p)*/
//                color_p++;
//            }
//        }

//    LCD_Address_Set(area->x1,area->y1,area->x2,area->y2); //<盖函数是设置LCD屏幕的扫描区域
//    HAL_DMA_Start_IT(&hdma_memtomem_dma2_channel7, (uint32_t)color_p, (uint32_t)&LCD->LCD_REG,
//                     ((area->x2+1) - area->x1) * ((area->y2+1) - area->y1));

//    }
 LCD_ColorFill(area->x1,area->y1,area->x2+1,area->y2+1,(uint16_t*)color_p);
//LCD_Fill(area->x1,area->y2,area->x2,area->y2,color_p);



    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

//void LVGL_LCD_FSMC_DMA_pCallback(DMA_HandleTypeDef *_hdma)
//{
//    lv_disp_flush_ready(&disp_drv1);
//}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
