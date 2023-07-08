#include "lv_port_indev.h"
#include"myGui.h"

extern lv_indev_t * indev_keypad;
static void event_ledoff( lv_event_t *e )
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    beep(100);

}

static void event_led( lv_event_t *e )
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_RESET);
    beep(100);

}

static void event_fan( lv_event_t *e )
{
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
   beep(100);

}



 void mygui(void )
 {
     lv_group_t * group = lv_group_create();
     lv_group_set_default(group);
     lv_indev_set_group(indev_keypad, group);
        /*此处是对于按钮部件的描写*/
     lv_obj_t *but1=lv_btn_create(lv_scr_act());
     lv_obj_align(but1,LV_ALIGN_TOP_MID,0,20);
     lv_obj_set_size(but1,100,30);
     lv_obj_t *butoff=lv_btn_create(lv_scr_act());
     lv_obj_t *but=lv_btn_create(lv_scr_act());
     lv_obj_center(but);
     lv_obj_set_size(butoff,50,10);
     lv_obj_set_size(but,100,30);
     lv_obj_align(butoff,LV_ALIGN_CENTER,0,-40);
     /*此处是对于阈值描述*/

     lv_obj_t   *bar  = lv_bar_create(lv_scr_act());
     lv_obj_set_size( bar, 100, 10);
     lv_obj_set_style_anim_time( bar, 500, LV_STATE_DEFAULT );
     lv_bar_set_value( bar, hum, LV_ANIM_ON );
     lv_bar_set_range( bar, 0, 50 );
     lv_obj_t   *bar_label  =lv_label_create(lv_scr_act());
     lv_label_set_text_fmt(bar_label, "number:%d",hum);
     lv_obj_align( bar_label,LV_ALIGN_LEFT_MID,0,30);
     lv_obj_align( bar,LV_ALIGN_LEFT_MID,100,30);
     lv_obj_set_style_bg_color(bar,lv_color_hex(0x3e901d),LV_STATE_DEFAULT);
     /*此处是对于字符按键的描写*/
     lv_obj_t   *label  =lv_label_create(lv_scr_act());
     lv_obj_t   *label1  =lv_label_create(lv_scr_act());

     lv_label_set_text_fmt( label, "temperature:" );
     lv_obj_align(label,LV_ALIGN_BOTTOM_LEFT,0,-40);
     lv_label_set_text_fmt( label1, "Humidity value:") ;
     lv_obj_align(label1,LV_ALIGN_BOTTOM_LEFT,0,-20);

    /*此处对于按键描述的描写*/
     lv_obj_set_style_bg_color(but1, lv_color_hex(0xf09354), LV_STATE_FOCUS_KEY);
     lv_obj_set_style_bg_color(but, lv_color_hex(0xf09354), LV_STATE_FOCUS_KEY);
     lv_obj_set_style_bg_color(butoff, lv_color_hex(0xf09354), LV_STATE_FOCUS_KEY);
     lv_obj_set_style_bg_color(but1, lv_color_hex(0x59a869), LV_STATE_PRESSED);
     lv_obj_set_style_bg_color(but, lv_color_hex(0x59a869), LV_STATE_PRESSED);
     lv_obj_set_style_bg_color(butoff, lv_color_hex(0x59a869), LV_STATE_PRESSED);

     lv_obj_t   *butlabel  =lv_label_create(lv_scr_act());
     lv_obj_t   *butlabel1  =lv_label_create(lv_scr_act());
     lv_obj_t   *off_all  =lv_label_create(lv_scr_act());
     lv_label_set_text_fmt( butlabel , "LED:");
     lv_obj_set_style_text_font( butlabel, &lv_font_montserrat_20, LV_STATE_DEFAULT );
     lv_obj_align(butlabel,LV_ALIGN_TOP_LEFT,5,20);
     lv_obj_set_style_bg_color( butlabel, lv_color_hex(0x20eeac), LV_STATE_DEFAULT );
     lv_obj_set_style_bg_opa(butlabel,200,LV_STATE_DEFAULT);

     lv_label_set_text_fmt( butlabel1, "FAN:") ;
     lv_obj_set_style_text_font( butlabel1, &lv_font_montserrat_20, LV_STATE_DEFAULT );
     lv_obj_set_style_bg_color( butlabel1, lv_color_hex(0x6485f0), LV_STATE_DEFAULT );
     lv_obj_align(butlabel1,LV_ALIGN_CENTER,-90,0);
     lv_obj_set_style_bg_opa(butlabel1,200,LV_STATE_DEFAULT);

     lv_label_set_text_fmt( off_all , "off-all");
     lv_obj_set_style_text_font(  off_all, &lv_font_montserrat_20, LV_STATE_DEFAULT );
     lv_obj_align( off_all,LV_ALIGN_CENTER,-90,-40);
    /*此处是对于回调的描写*/

     lv_obj_add_event(but1,event_led,LV_EVENT_PRESSED,NULL);
     lv_obj_add_event(butoff,event_ledoff,LV_EVENT_PRESSED,NULL);
     lv_obj_add_event(but,event_fan,LV_EVENT_PRESSED,NULL);

 }
void beep(unsigned int time)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_Delay(time);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}

