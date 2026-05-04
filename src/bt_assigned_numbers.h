#ifndef BT_ASSIGNED_NUMBERS_H
#define BT_ASSIGNED_NUMBERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *bt_assigned_company_name(uint16_t company_id);
const char *bt_assigned_ad_type_name(uint8_t ad_type);

#ifdef __cplusplus
}
#endif

#endif /* BT_ASSIGNED_NUMBERS_H */
