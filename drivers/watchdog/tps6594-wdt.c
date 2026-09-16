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
 * Window-1 / Window-2 bracket the four-byte Q&A handshake itself, not the
 *   user-visible timeout, and are set to maximum (~70ms each) so the answer
 *   writes tolerate scheduling jitter.
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

/* WD_THR_CFG bits — datasheet ZHCSKK3B §8.7.1.193 */
#define WD_THR_RST_EN			BIT(7)
#define WD_THR_WD_EN			BIT(6)
#define WD_THR_FAIL_TH_SHIFT		3
#define WD_THR_FAIL_TH_MASK		GENMASK(5, 3)
#define WD_THR_RST_TH_MASK		GENMASK(2, 0)

/* WD_ERR_STATUS bits — all R/W1C */
#define WD_ERR_RST_INT			BIT(7)
#define WD_ERR_FAIL_INT			BIT(6)
#define WD_ERR_ANSW_ERR			BIT(5)
#define WD_ERR_SEQ_ERR			BIT(4)
#define WD_ERR_ANSW_EARLY		BIT(3)
#define WD_ERR_TRIG_EARLY		BIT(2)
#define WD_ERR_TIMEOUT			BIT(1)
#define WD_ERR_LONGWIN_TIMEOUT		BIT(0)
#define WD_ERR_ALL			GENMASK(7, 0)

/* Anything here means the last Q&A sequence did not complete cleanly. */
#define WD_ERR_SEQ_FAIL			(WD_ERR_ANSW_ERR | WD_ERR_SEQ_ERR | \
					 WD_ERR_ANSW_EARLY | WD_ERR_TRIG_EARLY | \
					 WD_ERR_TIMEOUT)

/* WD_QUESTION_ANSW_CNT bits */
#define WD_QA_QUESTION_MASK		GENMASK(3, 0)

/* WD_WINx_CFG bits */
#define WD_WIN_CFG_MASK			GENMASK(6, 0)

/*
 * Windows at maximum: t = (WIN_VAL + 1) * 0.55 ms +-5%, so 0x7f -> 66.9-73.9 ms.
 * Answer-0 must land in Window-2, i.e. between 73.9 and 133.8 ms worst case;
 * 100 ms sits mid-slot with ~26 ms slack either side.
 */
#define TPS6594_WIN1_VAL		0x7f
#define TPS6594_WIN2_VAL		0x7f
#define TPS6594_W1_SLEEP_MS		100

/*
 * NVM defaults, left as-is. They have no effect here: the device clears
 * WD_FAIL_CNT whenever the watchdog re-enters the Long Window (§8.7.1.194),
 * and WD_RETURN_LONGWIN puts it back there after every sequence, so FAIL_TH
 * and RST_TH are never reached. The reset path in this mode is the long-window
 * timeout alone — a bad sequence is visible in WD_ERR_STATUS but does not by
 * itself force a reset.
 */
#define TPS6594_FAIL_TH			7
#define TPS6594_RST_TH			7

/*
 * Retries for a write to a register that is locked outside the Long Window.
 * Spaced a window pair apart so a sequence in flight has returned by then.
 */
#define TPS6594_LOCKED_WRITE_TRIES	4

#define TPS6594_WDT_MIN_TIMEOUT		1
#define TPS6594_WDT_MAX_TIMEOUT		770
#define TPS6594_WDT_DEFAULT_TIMEOUT	32

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

static int tps6594_wdt_answer(struct tps6594_wdt *wdt, const u8 *ans, int n)
{
	int i, ret;

	for (i = 0; i < n; i++) {
		ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_ANSWER, ans[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * One feed, two rounds.
 *
 * Per §8.3.11.3 the four WD_ANSWER writes that take the watchdog out of the
 * Long Window are the exit handshake; the first watchdog sequence only starts
 * after them. A feed therefore has to answer twice: four writes to get out,
 * then the sequence itself — Answer-3/2/1 inside Window-1, Answer-0 inside
 * Window-2. Writing only the first four (as this driver used to) leaves the
 * sequence unanswered, which measures as WD_SEQ_ERR at +70 ms and WD_TIMEOUT
 * at +140 ms with WD_QUESTION never advancing.
 *
 * WD_RETURN_LONGWIN stays asserted, so the sequence ends back in the Long
 * Window with its timeout restarted — that is what makes the user-visible
 * timeout the long-window value rather than a window pair.
 */
static int tps6594_wdt_do_qa(struct tps6594_wdt *wdt)
{
	const u8 *ans;
	int mode, q, ret;

	mode = tps6594_wdt_read(wdt, TPS6594_WDG_WD_MODE_REG);
	if (mode < 0)
		return mode;
	mode |= WD_MODE_RETURN_LONGWIN;
	mode &= ~WD_MODE_PWRHOLD;
	ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_MODE_REG, (u8)mode);
	if (ret)
		return ret;

	/* Exit handshake — untimed, the Long Window has no window pair */
	q = tps6594_wdt_read(wdt, TPS6594_WDG_WD_QUESTION_ANSW);
	if (q < 0)
		return q;
	ret = tps6594_wdt_answer(wdt, tps6594_qa_table[q & WD_QA_QUESTION_MASK], 4);
	if (ret)
		return ret;

	/* Window-1 of the first sequence is running from here */
	q = tps6594_wdt_read(wdt, TPS6594_WDG_WD_QUESTION_ANSW);
	if (q < 0)
		return q;
	ans = tps6594_qa_table[q & WD_QA_QUESTION_MASK];
	ret = tps6594_wdt_answer(wdt, ans, 3);
	if (ret)
		return ret;

	msleep(TPS6594_W1_SLEEP_MS);

	return tps6594_wdt_write(wdt, TPS6594_WDG_WD_ANSWER, ans[3]);
}

/*
 * WD_WIN1/WIN2/LONGWIN, WD_EN and the threshold fields are write-locked while
 * the watchdog sits outside the Long Window (§8.3.11.3), and the PMIC ACKs the
 * locked write on I2C before discarding it. Write, read back, and retry a
 * window pair apart, so a dropped write becomes an error instead of a
 * configuration that silently stayed at its previous value. Measured: a write
 * issued ~1 ms after Answer-0 is dropped, one issued ~20 ms later lands.
 */
static int tps6594_wdt_write_verify(struct tps6594_wdt *wdt, u8 reg, u8 val)
{
	int ret, i;

	for (i = 0; i < TPS6594_LOCKED_WRITE_TRIES; i++) {
		ret = tps6594_wdt_write(wdt, reg, val);
		if (ret)
			return ret;

		ret = tps6594_wdt_read(wdt, reg);
		if (ret < 0)
			return ret;
		if (ret == val)
			return 0;

		msleep(TPS6594_W1_SLEEP_MS);
	}

	dev_err(&wdt->client->dev,
		"reg 0x%02x kept rejecting 0x%02x (reads 0x%02x)\n",
		reg, val, ret);
	return -EIO;
}

/*
 * Configure the write-locked registers. Must be called only with the Long
 * Window freshly restarted -- see the hazard note in tps6594_wdt_start().
 */
static int tps6594_wdt_configure_locked_regs(struct tps6594_wdt *wdt,
					     unsigned int timeout_s)
{
	u8 thr_cfg;
	int ret;

	ret = tps6594_wdt_write_verify(wdt, TPS6594_WDG_WD_WIN1_CFG,
				       TPS6594_WIN1_VAL & WD_WIN_CFG_MASK);
	if (ret)
		return ret;
	ret = tps6594_wdt_write_verify(wdt, TPS6594_WDG_WD_WIN2_CFG,
				       TPS6594_WIN2_VAL & WD_WIN_CFG_MASK);
	if (ret)
		return ret;
	ret = tps6594_wdt_write_verify(wdt, TPS6594_WDG_WD_LONGWIN_CFG,
				       tps6594_secs_to_longwin(timeout_s));
	if (ret)
		return ret;

	thr_cfg = WD_THR_RST_EN | WD_THR_WD_EN |
		  ((TPS6594_FAIL_TH << WD_THR_FAIL_TH_SHIFT) & WD_THR_FAIL_TH_MASK) |
		  (TPS6594_RST_TH & WD_THR_RST_TH_MASK);
	return tps6594_wdt_write_verify(wdt, TPS6594_WDG_WD_THR_CFG, thr_cfg);
}

static int tps6594_wdt_start(struct watchdog_device *wdd)
{
	struct tps6594_wdt *wdt = watchdog_get_drvdata(wdd);
	struct device *dev = &wdt->client->dev;
	int thr, ret;

	thr = tps6594_wdt_read(wdt, TPS6594_WDG_WD_THR_CFG);
	if (thr < 0)
		return thr;

	/*
	 * If the PMIC already has the watchdog enabled — the NVM default, and
	 * still true across a warm reset — it has been sitting in the Long
	 * Window for an unknown time, quite possibly longer than the timeout we
	 * are about to program. Shortening WD_LONGWIN_CFG below the time
	 * already spent in the window kills the SoC on the spot: the serial
	 * console goes silent inside 21 ms, reproducible with that single
	 * write. (§8.3.11.2 promises only that the time-out stops working;
	 * measured silicon is far worse.) One complete Q&A sequence restarts
	 * the Long Window counter from zero, which makes the write safe — so
	 * demand proof that the sequence was clean before trusting it, and let
	 * the PMIC finish returning to the Long Window first, since a write
	 * issued immediately after Answer-0 is silently dropped instead.
	 *
	 * Deliberately do not clear WD_EN to dodge all this. That would leave
	 * the SoC unprotected across the following writes and, on any error
	 * return below, disarmed for good. Failing with the watchdog still
	 * armed at its NVM window is the safe direction to fail in.
	 */
	if (thr & WD_THR_WD_EN) {
		ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_ERR_STATUS,
					WD_ERR_ALL);
		if (ret)
			return ret;

		ret = tps6594_wdt_do_qa(wdt);
		if (ret)
			return ret;

		ret = tps6594_wdt_read(wdt, TPS6594_WDG_WD_ERR_STATUS);
		if (ret < 0)
			return ret;
		if (ret & WD_ERR_SEQ_FAIL) {
			dev_err(dev,
				"Q&A sequence failed (WD_ERR_STATUS=0x%02x); refusing to reprogram the Long Window\n",
				ret);
			return -EIO;
		}

		msleep(TPS6594_W1_SLEEP_MS);
	} else {
		/*
		 * WD_PWRHOLD must be clear before WD_EN goes back to 1
		 * (datasheet §8.3.11.2), and the PMIC sets it itself when the
		 * DISABLE_WDOG pin was asserted at reset. Mo 68A pulls that
		 * pin low today, but a board that straps it would otherwise
		 * come up with the watchdog parked in the Long Window: opened,
		 * active, reporting no error, and never biting.
		 */
		ret = tps6594_wdt_read(wdt, TPS6594_WDG_WD_MODE_REG);
		if (ret < 0)
			return ret;
		ret = tps6594_wdt_write(wdt, TPS6594_WDG_WD_MODE_REG,
					(u8)ret & ~WD_MODE_PWRHOLD);
		if (ret)
			return ret;
	}

	return tps6594_wdt_configure_locked_regs(wdt, wdd->timeout);
}

static int tps6594_wdt_stop(struct watchdog_device *wdd)
{
	struct tps6594_wdt *wdt = watchdog_get_drvdata(wdd);
	int thr;

	/*
	 * Clearing WD_EN halts the Long Window timer, but WD_EN is on the
	 * locked-register list, so a stop issued right after a ping can be
	 * ACKed and dropped. That failure direction is the dangerous one —
	 * userspace closed the device with the magic character and believes
	 * the watchdog is disarmed, while it is still counting — so verify
	 * the readback rather than trust the write.
	 */
	thr = tps6594_wdt_read(wdt, TPS6594_WDG_WD_THR_CFG);
	if (thr < 0)
		return thr;
	return tps6594_wdt_write_verify(wdt, TPS6594_WDG_WD_THR_CFG,
					(u8)(thr & ~WD_THR_WD_EN));
}

static int tps6594_wdt_ping(struct watchdog_device *wdd)
{
	return tps6594_wdt_do_qa(watchdog_get_drvdata(wdd));
}

static int tps6594_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int t)
{
	unsigned int old = wdd->timeout;
	int ret;

	/*
	 * Poking WD_LONGWIN_CFG from here carries exactly the hazard described
	 * in tps6594_wdt_start(): between pings the watchdog is idle in the
	 * Long Window, so a value shorter than the time already elapsed there
	 * kills the SoC — and this path is reachable from userspace through
	 * WDIOC_SETTIMEOUT. Go through start(), which restarts the window
	 * first. While inactive there is nothing to program: start() picks
	 * wdd->timeout up when userspace opens the device.
	 */
	wdd->timeout = t;
	if (!watchdog_active(wdd))
		return 0;

	ret = tps6594_wdt_start(wdd);
	if (ret)
		wdd->timeout = old;

	return ret;
}

static const struct watchdog_info tps6594_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE |
		   WDIOF_CARDRESET,
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

	/*
	 * WD_RST_INT and WD_LONGWIN_TIMEOUT_INT sit in the PMIC's always-on
	 * LDOVRTC domain (§8.4.1) and survive the warm reset they cause.
	 * Long-window expiry is the path this driver takes: WD_RETURN_LONGWIN
	 * clears WD_FAIL_CNT before it can reach the WD_RST_INT threshold.
	 * Latch either, then clear the page (R/W1C).
	 */
	ret = tps6594_wdt_read(wdt, TPS6594_WDG_WD_ERR_STATUS);
	if (ret < 0)
		return dev_err_probe(dev, ret, "WD_ERR_STATUS unreadable\n");
	dev_info(dev, "WD_ERR_STATUS at probe: 0x%02x\n", ret);
	if (ret & (WD_ERR_RST_INT | WD_ERR_LONGWIN_TIMEOUT))
		wdt->wdd.bootstatus |= WDIOF_CARDRESET;
	if (ret & WD_ERR_ANSW_ERR)
		dev_warn(dev, "stale WD_ANSW_ERR: previous answers were wrong\n");
	tps6594_wdt_write(wdt, TPS6594_WDG_WD_ERR_STATUS, WD_ERR_ALL);

	wdt->wdd.info = &tps6594_wdt_info;
	wdt->wdd.ops = &tps6594_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = TPS6594_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = TPS6594_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = TPS6594_WDT_DEFAULT_TIMEOUT;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, timeout, dev);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_stop_on_unregister(&wdt->wdd);

	/*
	 * Deliberately do not touch WD_LONGWIN_CFG here. The PMIC keeps the
	 * Long Window counting across a warm reset, so by the time we probe,
	 * more of it has elapsed than any timeout we would program — and
	 * shortening it below the elapsed time kills the SoC (datasheet
	 * §8.3.11.2 says only that the timeout stops working; measured
	 * behaviour is worse). The NVM's 772 s window stands until userspace
	 * opens the device, at which point start() reprograms it from Long
	 * Window with a freshly restarted timer.
	 */
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
