/* gs130-version: print the SDK version and build platform. Minimal reference sample. */
#include "gs130.h"
#include <stdio.h>

int main(void)
{
    printf("gs130_sdk %s (platform: %s)\n", gs130_version(), gs130_platform());
    return 0;
}
