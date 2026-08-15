#ifndef RNE_PLUGIN_SDK_H
#define RNE_PLUGIN_SDK_H

#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#define RNE_PLUGIN_EXPORT __declspec(dllexport)
#define RNE_PLUGIN_CALL __cdecl
#else
#define RNE_PLUGIN_EXPORT __attribute__((visibility("default")))
#define RNE_PLUGIN_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RNE_PLUGIN_SDK_VERSION UINT32_C(1)
#define RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION UINT32_C(1)
#define RNE_PLUGIN_MIN_ABI_VERSION UINT32_C(2)
#define RNE_PLUGIN_ABI_VERSION UINT32_C(3)
#define RNE_PLUGIN_ABI_VERSION_V2 UINT32_C(2)

#define RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION (UINT64_C(1) << 0)
#define RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION (UINT64_C(1) << 1)
#define RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND (UINT64_C(1) << 2)
#define RNE_CONTROLLER_CAP_MULTI_ROBOT (UINT64_C(1) << 3)
#define RNE_CONTROLLER_KNOWN_CAPABILITY_MASK                                  \
  (RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION |                            \
   RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION |                            \
   RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND |                                \
   RNE_CONTROLLER_CAP_MULTI_ROBOT)

typedef struct RneJointPosition {
  const char *name;
  double position_rad;
} RneJointPosition;

typedef struct RneJointVelocity {
  const char *name;
  double velocity_rad_s;
} RneJointVelocity;

typedef struct RneJointObservationV3 {
  const char *robot_id;
  const char *name;
  double position_rad;
  double velocity_rad_s;
  uint8_t has_velocity;
  uint8_t reserved[7];
} RneJointObservationV3;

typedef struct RneJointVelocityV3 {
  const char *robot_id;
  const char *name;
  double velocity_rad_s;
} RneJointVelocityV3;

typedef struct RneControllerStepResultV3 {
  int32_t status;
  size_t output_count;
} RneControllerStepResultV3;

typedef uint32_t(RNE_PLUGIN_CALL *RnePluginAbiVersionFn)(void);
typedef const char *(RNE_PLUGIN_CALL *RnePluginNameFn)(void);
typedef uint64_t(RNE_PLUGIN_CALL *RnePluginCapabilitiesFn)(void);
typedef void *(RNE_PLUGIN_CALL *RneControllerCreateFn)(
    const char *joint, double target_rad, double gain,
    double max_velocity_rad_s, char *error, size_t error_capacity);
typedef void(RNE_PLUGIN_CALL *RneControllerDestroyFn)(void *handle);
typedef size_t(RNE_PLUGIN_CALL *RneControllerStepFn)(
    const void *handle, const RneJointPosition *observations,
    size_t observation_count, RneJointVelocity *output,
    size_t output_capacity);
typedef int32_t(RNE_PLUGIN_CALL *RneControllerConfigureV3Fn)(
    void *handle, uint64_t required_capabilities, char *error,
    size_t error_capacity);
typedef int32_t(RNE_PLUGIN_CALL *RneControllerResetV3Fn)(
    void *handle, uint64_t episode, uint64_t seed, uint64_t step,
    uint64_t sim_time_ticks, char *error, size_t error_capacity);
typedef RneControllerStepResultV3(RNE_PLUGIN_CALL *RneControllerStepV3Fn)(
    void *handle, uint64_t step, uint64_t sim_time_ticks,
    const RneJointObservationV3 *observations, size_t observation_count,
    RneJointVelocityV3 *output, size_t output_capacity, char *error,
    size_t error_capacity);
typedef int32_t(RNE_PLUGIN_CALL *RneControllerShutdownV3Fn)(
    void *handle, char *error, size_t error_capacity);

RNE_PLUGIN_EXPORT uint32_t RNE_PLUGIN_CALL rne_plugin_abi_version(void);
RNE_PLUGIN_EXPORT const char *RNE_PLUGIN_CALL rne_plugin_name(void);
RNE_PLUGIN_EXPORT uint64_t RNE_PLUGIN_CALL rne_plugin_capabilities(void);
RNE_PLUGIN_EXPORT void *RNE_PLUGIN_CALL rne_controller_create(
    const char *joint, double target_rad, double gain,
    double max_velocity_rad_s, char *error, size_t error_capacity);
RNE_PLUGIN_EXPORT void RNE_PLUGIN_CALL rne_controller_destroy(void *handle);
RNE_PLUGIN_EXPORT size_t RNE_PLUGIN_CALL rne_controller_step(
    const void *handle, const RneJointPosition *observations,
    size_t observation_count, RneJointVelocity *output,
    size_t output_capacity);
RNE_PLUGIN_EXPORT int32_t RNE_PLUGIN_CALL rne_controller_configure_v3(
    void *handle, uint64_t required_capabilities, char *error,
    size_t error_capacity);
RNE_PLUGIN_EXPORT int32_t RNE_PLUGIN_CALL rne_controller_reset_v3(
    void *handle, uint64_t episode, uint64_t seed, uint64_t step,
    uint64_t sim_time_ticks, char *error, size_t error_capacity);
RNE_PLUGIN_EXPORT RneControllerStepResultV3 RNE_PLUGIN_CALL
rne_controller_step_v3(
    void *handle, uint64_t step, uint64_t sim_time_ticks,
    const RneJointObservationV3 *observations, size_t observation_count,
    RneJointVelocityV3 *output, size_t output_capacity, char *error,
    size_t error_capacity);
RNE_PLUGIN_EXPORT int32_t RNE_PLUGIN_CALL rne_controller_shutdown_v3(
    void *handle, char *error, size_t error_capacity);

#ifdef __cplusplus
}
#endif

#endif
