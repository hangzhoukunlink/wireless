/*
 * 	miaofng@2011 initial version
 */

#ifndef __VSEP_H_
#define __VSEP_H_

const char *vsep_desc[] = {
	"OK",
	"Open load",
	"Short to Ground",
	"Short to Battery"
};

nest_reg_t vsep_regs[] = {
	{.ofs = 0x00, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH01", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x02, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH02", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x04, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH03", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x06, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH04", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x08, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH05", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x0a, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH06", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x0c, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH07", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x0e, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH08", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x10, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH09", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x12, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH10", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x14, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH11", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x16, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH12", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x18, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH13", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x1a, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH14", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x1c, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH15", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x1e, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH16", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x20, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH17", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x22, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH18", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x24, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH19", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x26, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH20", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x28, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH21", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x2a, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH22", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x2c, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH23", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x2e, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH24", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x30, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH25", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x32, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH26", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x34, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH27", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x36, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH28", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x38, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH29", .bind = NULL, .desc = vsep_desc},
	{.ofs = 0x3a, .len = 0x02, .msk = 0x03, .val = 0x00, .name = "PCH30", .bind = NULL, .desc = vsep_desc},
};

nest_chip_t vsep = {
	.name = "VSEP",
	.regs = (nest_reg_t *) &vsep_regs,
	.nr_of_regs = 30,
};

#define vsep_init() nest_chip_init(&vsep)
#define vsep_bind(reg, pin) nest_chip_bind(&vsep, reg, pin)
#define vsep_mask(reg) nest_chip_mask(&vsep, reg, 0x00)
#define vsep_trap(reg, msk, val) nest_chip_trap(&vsep, reg, msk, val)
#define vsep_verify(data) nest_chip_verify(&vsep, data)

#endif
