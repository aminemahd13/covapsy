#include <assert.h>
#include <string.h>

#include "fw_app.h"
#include "fw_protocol.h"

static void test_cmd_still_parses(void)
{
    fw_drive_command_t cmd;
    assert(FwProtocol_ParseCommandLine("CMD,42,12.0,0.75,1,0", &cmd));
    assert(cmd.seq == 42u);
    assert(cmd.flags == FW_CMD_FLAG_RUN_ENABLE);
    assert(cmd.emergency_brake == false);
}

static void test_lcd_parse_valid_and_invalid(void)
{
    fw_lcd_frame_t lcd;
    assert(FwProtocol_ParseLcdLine(
        "LCD,5,MODE RACE|BR RUN|ST OK|VEL 0.50/0.60",
        &lcd));
    assert(lcd.seq == 5u);
    assert(strcmp(lcd.lines[0], "MODE RACE") == 0);
    assert(strcmp(lcd.lines[3], "VEL 0.50/0.60") == 0);

    assert(!FwProtocol_ParseLcdLine("LCD,1,ONLY|THREE|LINES", &lcd));
    assert(!FwProtocol_ParseLcdLine(
        "LCD,1,THIS_LINE_IS_TOO_LONG_123|B|C|D",
        &lcd));
    assert(!FwProtocol_ParseLcdLine(
        "LCD,1,BAD,COMMA|B|C|D",
        &lcd));
}

static void test_app_lcd_sequence_ordering(void)
{
    fw_app_t app;
    fw_lcd_frame_t latest;

    FwApp_Init(&app, 0u);
    assert(FwApp_OnCommandLine(&app, "LCD,10,NEW|LINE2|LINE3|LINE4", 1u));
    assert(FwApp_OnCommandLine(&app, "LCD,9,OLD|OLD|OLD|OLD", 2u));
    assert(FwApp_GetLatestLcdFrame(&app, &latest));
    assert(latest.seq == 10u);
    assert(strcmp(latest.lines[0], "NEW") == 0);
}

int main(void)
{
    test_cmd_still_parses();
    test_lcd_parse_valid_and_invalid();
    test_app_lcd_sequence_ordering();
    return 0;
}
