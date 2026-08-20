#include "rne_plugin_sdk.h"

#ifdef __cplusplus
#define RNE_STATIC_ASSERT static_assert
#else
#define RNE_STATIC_ASSERT _Static_assert
#endif

RNE_STATIC_ASSERT(sizeof(void *) == 8, "release ABI requires 64-bit pointers");
RNE_STATIC_ASSERT(sizeof(RneJointPosition) == 16, "RneJointPosition layout");
RNE_STATIC_ASSERT(offsetof(RneJointPosition, position_rad) == 8,
                  "position offset");
RNE_STATIC_ASSERT(sizeof(RneJointObservationV3) == 40,
                  "observation layout");
RNE_STATIC_ASSERT(offsetof(RneJointObservationV3, reserved) == 33,
                  "reserved offset");
RNE_STATIC_ASSERT(sizeof(RneControllerStepResultV3) == 16, "result layout");
RNE_STATIC_ASSERT(offsetof(RneControllerStepResultV3, output_count) == 8,
                  "count offset");

int main(void) { return RNE_PLUGIN_ABI_VERSION == 3 ? 0 : 1; }
