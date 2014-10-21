#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/syslog/ramlog.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

int main(int argc, char **argv) {
    int i = 0;
    while (1) {
        lldbg("hello world: %d\n", i++);
        asm volatile("nop");
    }
    return 0;
}
