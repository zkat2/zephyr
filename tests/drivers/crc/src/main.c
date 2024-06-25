/*
 * Copyright (c) 2024 Brill Power Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/crc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

/* Test cases for CRC_STM32 driver */

#define WAIT_THREAD_STACK_SIZE 1024
#define WAIT_THREAD_PRIO       -10

static void wait_thread_entry(void *a, void *b, void *c);

K_THREAD_STACK_DEFINE(wait_thread_stack_area, WAIT_THREAD_STACK_SIZE);
struct k_thread wait_thread_data;

/**
 * 1) Take the mutex lock
 * 2) Sleep for 50 ms (to allow ztest main thread to attempt to acquire lock)
 * 3) Release the mutex lock
 */
static void wait_thread_entry(void *a, void *b, void *c)
{
	UNUSED(a);
	UNUSED(b);
	UNUSED(c);

	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {.type = CRC8_CCITT,
			      .flags = 0,
			      .polynomial = CRC8_CCITT_POLY,
			      .initial_value = CRC8_CCITT_INIT_VAL};

	crc_begin(dev, &ctx);
	k_sleep(K_MSEC(50));
	crc_finish(dev, &ctx, data, sizeof(data));
}

/**
 * @brief Test that crc_8_ccitt works
 */
ZTEST(crc, test_crc_8_ccitt)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {.type = CRC8_CCITT,
			      .flags = 0,
			      .polynomial = CRC8_CCITT_POLY,
			      .initial_value = CRC8_CCITT_INIT_VAL};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_finish(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_verify(dev, &ctx, 0x4D), 0);
}

/**
 * @brief Test that crc_32_ieee works
 */
ZTEST(crc, test_crc_32_ieee_remain_0)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));
	uint32_t flags = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT;

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {.type = CRC32_IEEE,
			      .flags = flags,
			      .polynomial = CRC32_IEEE_POLY,
			      .initial_value = CRC32_IEEE_INIT_VAL};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_finish(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_verify(dev, &ctx, 0xCEA4A6C2), 0);
}

/**
 * @brief Test that crc_32_ieee works with a single byte
 *		remaining after the main process loop
 */
ZTEST(crc, test_crc_32_ieee_remain_1)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));
	uint32_t flags = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT;

	uint8_t data[9] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3B};

	struct crc_ctx ctx = {.type = CRC32_IEEE,
			      .flags = flags,
			      .polynomial = CRC32_IEEE_POLY,
			      .initial_value = CRC32_IEEE_INIT_VAL};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_finish(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_verify(dev, &ctx, 0x16AD0193), 0);
}

/**
 * @brief Test that crc_32_ieee works with two bytes
 *		remaining after the main process loop
 */
ZTEST(crc, test_crc_32_ieee_remain_2)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));
	uint32_t flags = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT;

	uint8_t data[10] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3B, 0x78};

	struct crc_ctx ctx = {.type = CRC32_IEEE,
			      .flags = flags,
			      .polynomial = CRC32_IEEE_POLY,
			      .initial_value = CRC32_IEEE_INIT_VAL};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_finish(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_verify(dev, &ctx, 0xE5CC797C), 0);
}

/**
 * @brief Test that crc_32_ieee works with three bytes
 *		remaining after the main process loop
 */
ZTEST(crc, test_crc_32_ieee_remain_3)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));
	uint32_t flags = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT;

	uint8_t data[11] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3B, 0x78, 0xB6};

	struct crc_ctx ctx = {.type = CRC32_IEEE,
			      .flags = flags,
			      .polynomial = CRC32_IEEE_POLY,
			      .initial_value = CRC32_IEEE_INIT_VAL};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_finish(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_verify(dev, &ctx, 0xA956085A), 0);
}

/**
 * @brief Test that CRC function returns an error when attempting to acquire lock
 *		from a thread that does not own the mutex
 */
ZTEST(crc, test_crc_threadsafe)
{
	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(crc));
	uint8_t data[11] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3B, 0x78, 0xB6};
	uint32_t flags = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT;
	struct crc_ctx ctx = {.type = CRC32_IEEE,
			      .flags = flags,
			      .polynomial = CRC32_IEEE_POLY,
			      .initial_value = CRC32_IEEE_INIT_VAL};

	/**
	 * Create new thread that will immediately take the mutex lock
	 */
	k_tid_t wait_tid =
		k_thread_create(&wait_thread_data, wait_thread_stack_area,
				K_THREAD_STACK_SIZEOF(wait_thread_stack_area), wait_thread_entry,
				NULL, NULL, NULL, WAIT_THREAD_PRIO, 0, K_NO_WAIT);

	UNUSED(wait_tid);

	/**
	 * Sleep for 10 ms to ensure that new thread has taken lock
	 */
	k_sleep(K_MSEC(10));

	/**
	 * Attempt to take lock, we can't zassert here else we'll exit all threads
	 * and the new thread will not release the lock, causing future tests to fail
	 */
	int setup_result = crc_begin(dev, &ctx); /* Should fail because device is locked */
	int finish_result = crc_finish(dev, &ctx, data, sizeof(data)); /* Should fail since ctx not
									  in progress */

	/**
	 * Wait for new thread to release lock
	 */
	k_sleep(K_MSEC(60));

	/**
	 * Test that crc_begin returned an error
	 */
	zassert_not_equal(setup_result, 0);
	zassert_not_equal(finish_result, 0);
}

/*******************************************************************************
TEST SUITE CREATION
*******************************************************************************/

ZTEST_SUITE(crc, NULL, NULL, NULL, NULL, NULL);
