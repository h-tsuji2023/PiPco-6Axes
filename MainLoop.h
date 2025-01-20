#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <inttypes.h>

extern "C"
{
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "class/cdc/cdc_device.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "hardware/clocks.h"

  // その他のPico SDKヘッダーファイル
}


#include "MachineControl.h"
#include "G-CodeInterpreter.h"
#include "PulseControl.h"


#include "MachineControl.h"

#endif

