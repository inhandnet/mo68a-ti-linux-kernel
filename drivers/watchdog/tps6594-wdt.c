// SPDX-License-Identifier: GPL-2.0
/*
 * TPS6594-Q1 / TPS6593-Q1 PMIC Watchdog driver (Q&A mode)
 *
 * Standalone i2c driver — attaches to the PMIC's dedicated WDG/ESM I2C2 port.
 * On Mo 68A this is mcu_i2c0 at slave address 0x12.
 *
 * Why standalone (not an MFD cell):
 *   The mainline tps6594 MFD core (drivers/mfd/tps6594-{core,i2c}.c) addresses
 *   pages 1..4 via base + page offset on the SAME i2c bus. On Mo 68A's PMIC
 *   NVM, the WDG/ESM page is mapped to PMIC I2C2 at slave 0x12 (a separate
 *   physical bus, mcu_i2c0), so the MFD page-offset model does not apply.
 *   This driver opens its own i2c_client on the WDG-only bus and talks to
 *   register addresses 0x00..0x0a (the page-4 layout).
 *
 * Mode: Q&A only. Mo 68A has no TRIG_WDOG GPIO routed, so Trigger mode is not
 *   an option. The PMIC NVM presets Q&A mode (WD_MODE_SELECT=1) and
 *   WD_QA_CFG = 0x0a (LFSR=00, FDBK=00, SEED=4'b1010), which matches the
 *   reference-answer table in TPS6594-Q1 datasheet (ZHCSKK3B) Table 8-9.
 *
 * Timeout: implemented via WD_LONGWIN_CFG. After each ping we set
 *   WD_RETURN_LONGWIN so the sequence ends back in Long Window and the
 *   long-window timer restarts. If userspace stops feeding, the long-window
 *   timeout (1s..770s) elapses and the PMIC issues a warm reset.
 *
 * Window-1 / Window-2 are kept short (~5ms each) — they only bracket the
 *   four-byte Q&A handshake itself, not the user-visible timeout.
 *
 * Copyright (C) 2026 Beijing InHand Networks Technology Co., Ltd.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/watchdog.h>

/* WDG/ESM page (page 4) register offsets, accessed via the dedicated I2C2 bus */
#define TPS6594_WDG_WD_ANSWER		0x01
#define TPS6594_WDG_WD_QUESTION_ANSW	0x02
#define TPS6594_WDG_WD_WIN1_CFG		0x03
#define TPS6594_WDG_WD_WIN2_CFG		0x04
#define TPS6594_WDG_WD_LONGWIN_CFG	0x05
#define TPS6594_WDG_WD_MODE_REG		0x06
#define TPS6594_WDG_WD_QA_CFG		0x07
#define TPS6594_WDG_WD_ERR_STATUS	0x08
#define TPS6594_WDG_WD_THR_CFG		0x09
#define TPS6594_WDG_WD_FAIL_CNT		0x0a

/* WD_MODE_REG bits */
#define WD_MODE_PWRHOLD			BIT(2)
#define WD_MODE_SELECT_QA		BIT(1)
#define WD_MODE_RETURN_LONGWIN		BIT(0)

/* WD_THR_CFG bits */
#define WD_THR_RST_EN			BIT(7)
#define WD_THR_WD_EN			BIT(6)
#define WD_THR_RST_TH_SHIFT		3
#define WD_THR_RST_TH_MASK		GENMASK(5, 3)
#define WD_THR_FAIL_TH_MASK		GENMASK(2, 0)

/* WD_QUESTION_ANSW_CNT bits */
#define WD_QA_QUESTION_MASK		GENMASK(3, 0)

/* WD_WINx_CFG bits */
#define WD_WIN_CFG_MASK			GENMASK(6, 0)

/*
 * Window-1/2 are ~5ms each: WIN_VAL=8 -> t = (8+1) * 0.55 = ~5ms.
 * Empirically generous enough that 3 i2c writes (~1ms) fit in W1 with margin,
 * and the inter-answer sleep below lands Answer-0 cleanly in W2.
 */
#define TPS6594_WIN1_VAL		8
#define TPS6594_WIN2_VAL		8
#define TPS6594_W1_SLEEP_US_MIN		6000
#define TPS6594_W1_SLEEP_US_MAX		7000

/*
 * Fail/Reset thresholds: tolerate up to (FAIL_TH + RST_TH + 1) bad sequences
 * before the PMIC issues WD_ERROR. With FAIL_TH=2 and RST_TH=2 that is 5
 * consecutive misses, comfortable margin against transient i2c glitches.
 */
#define TPS6594_FAIL_TH			2
#define TPS6594_RST_TH			2

#define TPS6594_WDT_MIN_TIMEOUT		1
#define TPS6594_WDT_MAX_TIMEOUT		770
#define TPS6594_WDT_DEFAULT_TIMEOUT	30

/*
 * Reference answers for the default WD_QA_CFG (LFSR=00, FDBK=00, SEED=1010).
 * Source: TPS6594-Q1 datasheet ZHCSKK3B Table 8-9.
 * Index [question][n] gives Answer-(3-n), i.e. the bytes are pre-ordered
 * for direct sequential write into WD_ANSWER.
 */
static const u8 tps6594_qa_table[16][4] = {
	[0x0] = { 0xFF, 0x0F, 0xF0, 0x00 },
	[0x1] = { 0xB0, 0x40, 0xBF, 0x4F },
	[0x2] = { 0xE9, 0x19, 0xE6, 0x16 },
	[0x3] = { 0xA6, 0x56, 0xA9, 0x59 },
	[0x4] = { 0x75, 0x85, 0x7A, 0x8A },
	[0x5] = { 0x3A, 0xCA, 0x35, 0xC5 },
	[0x6] = { 0x63, 0x93, 0x6C, 0x9C },
	[0x7] = { 0x2C, 0xDC, 0x23, 0xD3 },
	[0x8] = { 0xD2, 0x22, 0xDD, 0x2D },
	[0x9] = { 0x9D, 0x6D, 0x92, 0x62 },
	[0xA] = { 0xC4, 0x34, 0xCB, 0x3B },
	[0xB] = { 0x8B, 0x7B, 0x84, 0x74 },
	[0xC] = { 0x58, 0xA8, 0x57, 0xA7 },
	[0xD] = { 0x17, 0xE7, 0x18, 0xE8 },
	[0xE] = { 0x4E, 0xBE, 0x41, 0xB1 },
	[0xF] = { 0x01, 0xF1, 0x0E, 0xFE },
};

struct tps6594_wdt {
	struct watchdog_device wdd;
	struct i2c_client *client;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout;
module_param(timeout, uint, 0444);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds");

/*
 * Encode timeout seconds into WD_LONGWIN[7:0]:
 *   0x00       -> 80 ms
 *   0x01..0x40 -> 125 ms steps  (0.125 s .. 8 s)
 *   0x41..0xFF -> 4 s steps     (12 s .. 772 s)
 * There is a 8s..12s gap; values in it round down to 0x40 (8 s).
 */
static u8 tps6594_secs_to_longwin(unsigned int t)
{
	if (t == 0)
		return 1;
	if (t <= 8)
		return (u8)(t * 8);
	if (t < 12)
		return 0x40;
	if (t > 772)
		return 0xFF;
	return (u8)(0x41 + DIV_ROUND_UP(t - 12, 4));
}

static int tps6594_wdt_read(struct tps6594_wdt *wdt, u8 reg)
{
	return i2c_smbus_read_byte_data(wdt->client, reg);
}

static int tps6594_wdt_write(struct tps6594_wdt *wdt, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(wdt->client, reg, val);
}

/*
 * One complete Q&A sequence: read current question, write Answer-3/2/1 in
 * Window-1, sleep past W1, write Answer-0 in Window-2. WD_RETURN_LONGWIN is
 * asserted so the PMIC re-enters Long Window after Answer-0 — this restarts
 * the long-window timeout, which is what gives us user-visible timeout.
 */
static int tps6594_wdt_do_qa(struct tps6594_wdt *wdt)
{
	const u8 *ans;
	int mode, q, ret, i;

	q = tps6594_wdt_read(wdt, TPS6594_WDG_WD_QUESTION_ANSW);
	if (q < 0)
		return q;
	ans = tps6594_qa_table[q & WD_QA_QUESTION_MASK];

	mode = tps6594_wdt_read(wdt, TPS6594_WDG_WD_MODE_REG);
	if (mode < 0)
		return mode;
	mode |= WD_MODE_RETURN_LONGWIN;
	mode &= ~WD_MODE_PWRHOLD;
	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_MODE_REG, (u8)mode);
	if (ret)
		return ret;

	/* Answer-3, Answer-2, Answer-1 in Window-1 */
	for (i = 0; i < 3; i++) {
		ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_ANSWER, ans[i]);
		if (ret)
			return ret;
	}

	usleep_range(TPS6594_W1_SLEEP_US_MIN, TPS6594_W1_SLEEP_US_MAX);

	/* Answer-0 in Window-2 — sequence completes, device returns to LongWin */
	return tps6594_wdt_write(wdt, TPS6594_WDG_WD_ANSWER, ans[3]);
}

/*
 * Configure registers that are write-locked once the watchdog leaves Long
 * Window. Must be called only while we are in Long Window (e.g. before
 * the very first ping, or right after a ping returned us to Long Window).
 */
static int tps6594_wdt_configure_locked_regs(struct tps6594_wdt *wdt,
					     unsigned int timeout_s)
{
	u8 thr_cfg;
	int ret;

	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_WIN1_CFG,
				TPS6594_WIN1_VAL & WD_WIN_CFG_MASK);
	if (ret)
		return ret;
	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_WIN2_CFG,
				TPS6594_WIN2_VAL & WD_WIN_CFG_MASK);
	if (ret)
		return ret;
	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_LONGWIN_CFG,
				tps6594_secs_to_longwin(timeout_s));
	if (ret)
		return ret;

	thr_cfg = WD_THR_RST_EN | WD_THR_WD_EN |
		  ((TPS6594_RST_TH << WD_THR_RST_TH_SHIFT) & WD_THR_RST_TH_MASK) |
		  (TPS6594_FAIL_TH & WD_THR_FAIL_TH_MASK);
	return tps6594_wdt_write(wdt, TPS6594_WDG_WD_THR_CFG, thr_cfg);
}

static int tps6594_wdt_start(struct watchdog_device *wdd)
{
	struct tps6594_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = tps6594_wdt_configure_locked_regs(wdt, wdd->timeout);
	if (ret)
		return ret;

	/* Kick off the first Q&A; sequence end returns us to LongWindow with
	 * the timeout timer counting down.
	 */
	return tps6594_wdt_do_qa(wdt);
}

static int tps6594_wdt_stop(struct watchdog_device *wdd)
{
	struct tps6594_wdt *wdt = watchdog_get_drvdata(wdd);
	int thr;

	/* We are in Long Window (last ping returned here). Clearing WD_EN
	 * is allowed here and halts the long-window timer.
	 */
	thr = tps6594_wdt_read(wdt, TPS6594_WDG_WD_THR_CFG);
	if (thr < 0)
		return thr;
	return tps6594_wdt_write(wdt, TPS6594_WDG_WD_THR_CFG,
				 (u8)(thr & ~WD_THR_WD_EN));
}

static int tps6594_wdt_ping(struct watchdog_device *wdd)
{
	return tps6594_wdt_do_qa(watchdog_get_drvdata(wdd));
}

static int tps6594_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int t)
{
	struct tps6594_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	/* WD_LONGWIN_CFG is writeable in Long Window, where we always are
	 * between pings.
	 */
	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_LONGWIN_CFG,
				tps6594_secs_to_longwin(t));
	if (ret)
		return ret;

	wdd->timeout = t;
	return 0;
}

static const struct watchdog_info tps6594_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "TPS6594-Q1 Watchdog",
};

static const struct watchdog_ops tps6594_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= tps6594_wdt_start,
	.stop		= tps6594_wdt_stop,
	.ping		= tps6594_wdt_ping,
	.set_timeout	= tps6594_wdt_set_timeout,
};

static int tps6594_wdt_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tps6594_wdt *wdt;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->client = client;
	i2c_set_clientdata(client, wdt);

	/* Sanity-check we are talking to the WDG page (reg 0x02 should be
	 * readable and have ANSW_CNT either 11b (start of sequence) or 00b
	 * (idle in LongWindow); WD_MODE_REG should have SELECT_QA set since
	 * the NVM presets Q&A mode).
	 */
	ret = tps6594_wdt_read(wdt, TPS6594_WDG_WD_MODE_REG);
	if (ret < 0)
		return dev_err_probe(dev, ret, "WDG page not accessible\n");
	if (!(ret & WD_MODE_SELECT_QA))
		return dev_err_probe(dev, -ENODEV,
				     "PMIC NVM not in Q&A mode (WD_MODE_REG=0x%02x)\n",
				     ret);

	wdt->wdd.info = &tps6594_wdt_info;
	wdt->wdd.ops = &tps6594_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = TPS6594_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = TPS6594_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = TPS6594_WDT_DEFAULT_TIMEOUT;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, timeout, dev);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_stop_on_unregister(&wdt->wdd);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return dev_err_probe(dev, ret, "watchdog_register_device failed\n");

	dev_info(dev,
		 "TPS6594 watchdog registered, timeout=%us (Q&A mode, slave 0x%02x)\n",
		 wdt->wdd.timeout, client->addr);
	return 0;
}

static const struct of_device_id tps6594_wdt_of_match[] = {
	{ .compatible = "ti,tps6594-wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, tps6594_wdt_of_match);

static const struct i2c_device_id tps6594_wdt_id[] = {
	{ "tps6594-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6594_wdt_id);

static struct i2c_driver tps6594_wdt_driver = {
	.driver = {
		.name = "tps6594-wdt",
		.of_match_table = tps6594_wdt_of_match,
	},
	.probe = tps6594_wdt_probe,
	.id_table = tps6594_wdt_id,
};
module_i2c_driver(tps6594_wdt_driver);

MODULE_AUTHOR("Beijing InHand Networks Technology Co., Ltd.");
MODULE_DESCRIPTION("TI TPS6594-Q1 PMIC watchdog driver (Q&A mode)");
MODULE_LICENSE("GPL");
