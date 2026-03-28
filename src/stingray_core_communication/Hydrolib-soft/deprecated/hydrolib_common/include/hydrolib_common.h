#ifndef HYDROLIB_COMMON_H_
#define HYDROLIB_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HYDROLIB_RETURN_OK = 0,
    HYDROLIB_RETURN_FAIL,
    HYDROLIB_RETURN_NO_DATA,
    HYDROLIB_RETURN_BUSY,
    HYDROLIB_RETURN_ERROR
} hydrolib_ReturnCode;

#ifdef __cplusplus
}
#endif

#endif
