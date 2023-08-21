#include "key.h"

KEY_EVENT key_status_check(KEY *key, int interval) {
  KEY_EVENT key_event = KE_NONE;

  switch (key->status) {
    case KS_RELEASE: {
      key->count = 0;
      if (key->get() == K_PRESS) {
        key->status = KS_SHAKE;
      }
    } break;
    case KS_SHAKE: {
      if (key->get() == K_PRESS) {
        key->status = KS_PRESS;
        key_event = KE_PRESS;
      } else {
        key->status = KS_RELEASE;
      }
    } break;
    case KS_PRESS: {
      if (key->get() == K_PRESS) {
        key->count += 1;
        if (key->count == interval) {
          key_event = KE_LONG_PRESS;
        }
      } else {
        key->status = KS_RELEASE;
        if (interval <= 0) {
          key_event = KE_RELEASE;
        } else if (key->count < interval) {
          key_event = KE_RELEASE;
        } else {
          key_event = KE_LONG_RELEASE;
        }
      }
    } break;
  }

  return key_event;
}
