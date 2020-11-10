
#pragma once

_attribute_ram_code_ u8 analog_read(u8 addr);
_attribute_ram_code_ void analog_write(u8 addr, u8 v);
_attribute_ram_code_ void analog_read_blk(u8 addr, u8 *v, int len);
_attribute_ram_code_ void analog_write_blk(u8 addr, u8 *v, int len);




