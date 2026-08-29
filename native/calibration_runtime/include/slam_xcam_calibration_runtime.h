#ifndef SLAM_XCAM_CALIBRATION_RUNTIME_H
#define SLAM_XCAM_CALIBRATION_RUNTIME_H

#include <stdint.h>

#ifdef _WIN32
#ifdef SLAM_CAL_RUNTIME_BUILD
#define SLAM_CAL_API __declspec(dllexport)
#else
#define SLAM_CAL_API __declspec(dllimport)
#endif
#else
#define SLAM_CAL_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SLAM_CAL_ABI_VERSION 1u
#define SLAM_CAL_MODEL_2025 2025u
#define SLAM_CAL_MODEL_2026 2026u
#define SLAM_CAL_FLAG_DISTORTION_CORRECTION 1u

typedef void* SlamCalHandle;

SLAM_CAL_API uint32_t slam_cal_abi_version(void);
SLAM_CAL_API const char* slam_cal_runtime_version(void);
SLAM_CAL_API int32_t slam_cal_model_available(uint32_t model);

SLAM_CAL_API int32_t slam_cal_create(
    uint32_t model,
    uint32_t eye_size,
    uint32_t flags,
    float field_of_view_degrees,
    SlamCalHandle* output_handle
);

SLAM_CAL_API int32_t slam_cal_render_rgb24(
    SlamCalHandle handle,
    const uint8_t* input_sbs,
    uint32_t input_stride_bytes,
    const float* correction_matrix_3x3,
    const float* rolling_shutter_matrices_3x3,
    uint32_t rolling_shutter_row_count,
    uint8_t* output_sbs,
    uint32_t output_stride_bytes
);

SLAM_CAL_API void slam_cal_destroy(SlamCalHandle handle);
/* Optional ABI v1 extension. Returns the actual renderer used by this handle. */
SLAM_CAL_API const char* slam_cal_backend_name(SlamCalHandle handle);
SLAM_CAL_API const char* slam_cal_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
