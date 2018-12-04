
#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#ifdef __cplusplus
#define EXTERN_GLOBALS extern
#else
#define EXTERN_GLOBALS
#endif

EXTERN_GLOBALS float obsv_diff[6];
EXTERN_GLOBALS float ref_diff[9];

void updateAttitudeObserver();
void updateAttitudeController();

#endif
