#ifndef __KEY_H__
#define __KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  K_RELEASE,
  K_PRESS,
} KEY_VALUE;

typedef enum {
  KS_RELEASE,
  KS_SHAKE,
  KS_PRESS,
} KEY_STATUS;

typedef enum {
  KE_PRESS,
  KE_RELEASE,
  KE_LONG_PRESS,
  KE_LONG_RELEASE,
  KE_NONE,
} KEY_EVENT;

typedef struct {
  KEY_STATUS status;
  int count;
  KEY_VALUE (*get)();
} KEY;

KEY_EVENT key_status_check(KEY *key, int interval);

#ifdef __cplusplus
}
#endif

#endif
