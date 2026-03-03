#include "App.h"
#include <cstdio>

int main() {
    shitcad::App app;
    if (!app.init()) {
        fprintf(stderr, "Failed to initialize SHITcad\n");
        return 1;
    }
    app.run();
    app.shutdown();
    return 0;
}
