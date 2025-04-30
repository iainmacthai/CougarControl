SET(SOURCES ui_Screen1.c
    ui_Screen2.c
    ui_Screen3.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_innovatum_logo_email_png.c
    ui_img_cougarv6shape_png.c
    ui_img_blackthruster32x32_png.c
    ui_img_configicongrey_png.c)

add_library(ui ${SOURCES})
