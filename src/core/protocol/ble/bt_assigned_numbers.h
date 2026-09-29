#ifndef BT_ASSIGNED_NUMBERS_H
#define BT_ASSIGNED_NUMBERS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *bt_assigned_company_name(uint16_t company_id);
const char *bt_assigned_ad_type_name(uint8_t ad_type);

/* 16-bit Service / Service Class UUID name (GATT + Classic merged). */
const char *bt_assigned_service_uuid_name(uint16_t uuid);

/* GAP Appearance: category/subcategory decode (value = category<<6|sub). */
const char *bt_assigned_appearance_category(uint16_t appearance);
const char *bt_assigned_appearance_subcategory(uint16_t appearance);
void bt_assigned_appearance_format(uint16_t appearance, char *out, size_t cap);

/* Class of Device (AD 0x0D, 24-bit). */
const char *bt_cod_major_service_name(unsigned int bit);
const char *bt_cod_major_device_name(unsigned int major);
const char *bt_cod_minor_device_name(unsigned int major, unsigned int minor);
void bt_cod_format(uint32_t cod, char *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* BT_ASSIGNED_NUMBERS_H */
