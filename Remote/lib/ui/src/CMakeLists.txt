SET(SOURCES ui_comp_limbcontrol.c
    ui_comp.c
    ui_HomeScreen.c
    ui_LCScreen.c
    ui_WalkScreen.c
    ui_FLScreen.c
    ui_FRScreen.c
    ui_BLScreen.c
    ui_BRScreen.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c)

add_library(ui ${SOURCES})
