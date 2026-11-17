// =============================================================================
// COMPILE-CHECK STUB of the Stereolabs ZED SDK 4.x C++ API (sl/Camera.hpp).
//
// Declares ONLY the types / enumerators / members / methods that cmr_zed_cpp
// uses, with signatures copied from ZED SDK 4.1 (sl/Camera.hpp + sl/Core.hpp +
// sl_types). Nothing is defined: it is used only by the CMR_ZED_STUB_CHECK
// CMake option to compile (never link, never install) the ZED nodes on a
// machine without the SDK. Real builds use /usr/local/zed/include.
// =============================================================================
#ifndef CMR_ZED_CPP_STUB_SL_CAMERA_HPP_
#define CMR_ZED_CPP_STUB_SL_CAMERA_HPP_

#include <cstddef>
#include <vector>

// From cuda_runtime_api.h (the real header includes it); only appears as a
// defaulted trailing parameter of retrieveImage / retrieveMeasure.
typedef struct CUstream_st * cudaStream_t;

namespace sl
{

// ---- sl/types: basic vector types -------------------------------------------
typedef unsigned char uchar1;

template<typename T>
class Vector2
{
public:
  T x, y;
  Vector2();
  Vector2(const T x_, const T y_);
};

template<typename T>
class Vector3
{
public:
  T x, y, z;
  Vector3();
};

template<typename T>
class Vector4
{
public:
  T x, y, z, w;
  Vector4();
};

typedef Vector2<unsigned int> uint2;
typedef Vector3<float> float3;
typedef Vector4<float> float4;

// ---- sl::String ---------------------------------------------------------------
class String
{
public:
  const char * c_str() const;
};

// ---- enums --------------------------------------------------------------------
enum class ERROR_CODE
{
  SUCCESS = 0,
};
String toString(const ERROR_CODE & errorCode);

enum class MEM
{
  CPU = 1,
};

enum class MAT_TYPE
{
  F32_C1,
  F32_C2,
  F32_C3,
  F32_C4,
  U8_C1,
  U8_C2,
  U8_C3,
  U8_C4,
};

enum class RESOLUTION
{
  HD1080,
  HD720,
};

enum class DEPTH_MODE
{
  NEURAL,
};

enum class UNIT
{
  METER,
};

enum class COORDINATE_SYSTEM
{
  RIGHT_HANDED_Z_UP_X_FWD,
};

enum class VIEW
{
  LEFT,
  RIGHT,
};

enum class MEASURE
{
  DEPTH,  // only referenced as retrieveMeasure's default argument
  XYZ,
  XYZRGBA,
};

enum class POSITIONAL_TRACKING_STATE
{
  OFF,
  OK,
};

enum class POSITIONAL_TRACKING_MODE
{
  GEN_1,
};

enum class REFERENCE_FRAME
{
  WORLD,
};

enum class SPATIAL_MAP_TYPE
{
  MESH,
};

enum class PLANE_TYPE
{
  HORIZONTAL,
};

// ---- sl::Resolution -------------------------------------------------------------
struct Resolution
{
  size_t width;
  size_t height;
  Resolution(size_t w_ = 0, size_t h_ = 0);
};

// ---- sl::Mat --------------------------------------------------------------------
class Mat
{
public:
  Mat();
  ~Mat();

  ERROR_CODE alloc(Resolution resolution, MAT_TYPE mat_type, MEM memory_type = MEM::CPU);

  template<typename N>
  ERROR_CODE getValue(size_t x, size_t y, N * value, MEM memory_type = MEM::CPU) const;

  size_t getWidth() const;
  size_t getHeight() const;
  MAT_TYPE getDataType() const;
  size_t getStepBytes(MEM memory_type = MEM::CPU) const;

  template<typename N>
  N * getPtr(MEM memory_type = MEM::CPU) const;
};

// ---- positional tracking ----------------------------------------------------------
class Translation : public float3
{
};

class Pose
{
public:
  Pose();
  Translation getTranslation();
  float3 getRotationVector();
};

// ---- planes -------------------------------------------------------------------------
class Plane
{
public:
  Plane();
  PLANE_TYPE type;
  float4 getPlaneEquation();
  std::vector<float3> getBounds();
};

// ---- parameter structs ---------------------------------------------------------------
class InitParameters
{
public:
  RESOLUTION camera_resolution;
  int camera_fps;
  DEPTH_MODE depth_mode;
  UNIT coordinate_units;
  COORDINATE_SYSTEM coordinate_system;
  InitParameters();
};

class RuntimeParameters
{
public:
  RuntimeParameters();
};

class PositionalTrackingParameters
{
public:
  POSITIONAL_TRACKING_MODE mode;
  PositionalTrackingParameters();
};

class SpatialMappingParameters
{
public:
  SPATIAL_MAP_TYPE map_type;
  SpatialMappingParameters();
};

class PlaneDetectionParameters
{
public:
  float max_distance_threshold;
  float normal_similarity_threshold;
  PlaneDetectionParameters();
};

// ---- sl::Camera -----------------------------------------------------------------------
class Camera
{
public:
  Camera();
  ~Camera();

  ERROR_CODE open(InitParameters init_parameters = InitParameters());
  void close();
  ERROR_CODE grab(RuntimeParameters rt_parameters = RuntimeParameters());

  ERROR_CODE retrieveImage(
    Mat & mat, VIEW view = VIEW::LEFT, MEM type = MEM::CPU,
    Resolution image_size = Resolution(0, 0), cudaStream_t stream = 0);
  ERROR_CODE retrieveMeasure(
    Mat & mat, MEASURE measure = MEASURE::DEPTH,
    MEM type = MEM::CPU, Resolution image_size = Resolution(0, 0), cudaStream_t stream = 0);

  ERROR_CODE enablePositionalTracking(
    PositionalTrackingParameters tracking_parameters = PositionalTrackingParameters());
  POSITIONAL_TRACKING_STATE getPosition(
    Pose & camera_pose, REFERENCE_FRAME reference_frame = REFERENCE_FRAME::WORLD);

  ERROR_CODE enableSpatialMapping(
    SpatialMappingParameters spatial_mapping_parameters = SpatialMappingParameters());

  ERROR_CODE findPlaneAtHit(
    sl::uint2 coord, sl::Plane & plane,
    PlaneDetectionParameters plane_detection_parameters = PlaneDetectionParameters());
};

}  // namespace sl

#endif  // CMR_ZED_CPP_STUB_SL_CAMERA_HPP_
