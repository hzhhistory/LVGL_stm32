#include "lv_port_indev.h"
#include"myGui.h"
#include"key.h"
extern lv_indev_t * indev_keypad;

static void event_handler( lv_event_t *event)
{

}
 void mygui() {
     lv_group_t *group=lv_group_create();
     lv_indev_set_group(indev_keypad,group);
     lv_group_set_default(group);
     lv_obj_t *list = lv_list_create(lv_scr_act());
     lv_obj_t *btn = lv_list_add_btn(list, LV_SYMBOL_MINUS, "WLAN");
     lv_obj_t *btn0 = lv_list_add_btn(list, LV_SYMBOL_EYE_OPEN , "usb");
     lv_obj_t *btn1 = lv_list_add_btn(list, LV_SYMBOL_NEXT , "bluetooth");
     lv_obj_t *btn2 = lv_list_add_btn(list,LV_SYMBOL_PLAY, "power");


//    lv_group_t *g = lv_group_create();
//     lv_group_set_default(g);
//     lv_indev_set_group(indev_keypad, g);
//    lv_obj_t * switch1 = lv_btn_create( lv_scr_act());
//    lv_obj_set_size( switch1, 80, 20 );
//    lv_obj_set_align(switch1,LV_ALIGN_CENTER);
//    lv_obj_set_style_bg_color(switch1,lv_color_hex(0x0000a0),LV_STATE_DEFAULT);
//    lv_obj_add_event(switch1,event_handler,LV_KEY_ENTER,NULL);
//     lv_obj_t * switch2 = lv_btn_create( lv_scr_act());
//     lv_obj_set_size( switch2, 80, 20 );





 }