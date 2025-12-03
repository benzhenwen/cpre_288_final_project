#pragma once

#include "command.h"

// gen a command that just calls a function


void invoke_function_cmd_start_callback(CommandData * data) {
    data->functionPointer.function();
}

Command gen_invoke_function_cmd(void (*f)()) {
    CommandData cd;
    cd.functionPointer = (FunctionPointerCD) {f};

    Command c;
    c.on_start = &invoke_function_cmd_start_callback;
    c.is_complete = &always_true;
    c.is_interrupt = &always_false;
    c.data = cd;

    return c;
}
