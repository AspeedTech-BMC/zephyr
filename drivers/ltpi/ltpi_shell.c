/*
 * Copyright (c) 2025 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/ltpi.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ltpi_shell, CONFIG_LTPI_LOG_LEVEL);

/*
 * ltpi_do_link() can block for seconds while link training retries, and
 * may never return at all when called with timeout_ms=0. Calling it
 * directly from the shell's own command thread would make "ltpi init"
 * block the shell for that whole duration, with no way to run
 * "ltpi status" or anything else until it returns. Run it on a dedicated
 * background thread per device instead: "ltpi init" just starts it and
 * returns immediately, and "ltpi status" reports progress/result.
 */
#define LTPI_SHELL_MAX_DEV	2
#define LTPI_SHELL_STACK_SIZE	2048
/*
 * The shell's own thread runs at K_LOWEST_APPLICATION_THREAD_PRIO (see
 * subsys/shell/shell.c) specifically so it never starves other work.
 * A higher-priority background thread here would preempt and freeze the
 * shell for the whole link-training duration -- the opposite of what this
 * background thread exists for. Match it, and rely on ltpi_do_link()'s
 * internal poll loop yielding periodically (see ltpi_poll_link_mng_state())
 * so this thread and the shell actually take turns.
 */
#define LTPI_SHELL_THREAD_PRIO	K_LOWEST_APPLICATION_THREAD_PRIO

/*
 * Every LTPI node in this tree (both aspeed,ast27xx-ltpi and
 * aspeed,ast10x0-g2-ltpi) is named "ltpi@<addr>" from its devicetree node
 * name, which is also Zephyr's default device name absent an explicit
 * override. Restrict completion and lookup to that prefix: dev->api is
 * blindly cast to `struct ltpi_driver_api *` below, and doing that to an
 * arbitrary other device (e.g. "ltpi init uart0") is a type-confused
 * function-pointer call into whatever that device's real api struct holds.
 */
#define LTPI_DEVICE_PREFIX	"ltpi"

static bool is_ltpi_device(const struct device *dev)
{
	return strncmp(dev->name, LTPI_DEVICE_PREFIX, strlen(LTPI_DEVICE_PREFIX)) == 0;
}

enum ltpi_shell_state {
	LTPI_SHELL_IDLE = 0,
	LTPI_SHELL_RUNNING,
	LTPI_SHELL_DONE,
};

struct ltpi_shell_ctx {
	const struct device *dev;
	struct k_thread thread;
	atomic_t state;
	int timeout_ms;
	int result;
	bool started;
};

static struct ltpi_shell_ctx ltpi_shell_ctxs[LTPI_SHELL_MAX_DEV];
static K_THREAD_STACK_ARRAY_DEFINE(ltpi_shell_stacks, LTPI_SHELL_MAX_DEV, LTPI_SHELL_STACK_SIZE);

static struct ltpi_shell_ctx *ltpi_shell_ctx_find(const struct device *dev)
{
	for (int i = 0; i < LTPI_SHELL_MAX_DEV; i++) {
		if (ltpi_shell_ctxs[i].dev == dev) {
			return &ltpi_shell_ctxs[i];
		}
	}

	return NULL;
}

static struct ltpi_shell_ctx *ltpi_shell_ctx_find_or_alloc(const struct device *dev)
{
	int free_idx = -1;

	for (int i = 0; i < LTPI_SHELL_MAX_DEV; i++) {
		if (ltpi_shell_ctxs[i].dev == dev) {
			return &ltpi_shell_ctxs[i];
		}
		if (free_idx < 0 && !ltpi_shell_ctxs[i].dev) {
			free_idx = i;
		}
	}

	if (free_idx < 0) {
		return NULL;
	}

	ltpi_shell_ctxs[free_idx].dev = dev;
	atomic_set(&ltpi_shell_ctxs[free_idx].state, LTPI_SHELL_IDLE);

	return &ltpi_shell_ctxs[free_idx];
}

static void ltpi_link_thread_entry(void *p1, void *p2, void *p3)
{
	struct ltpi_shell_ctx *ctx = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ctx->result = ltpi_do_link(ctx->dev, ctx->timeout_ms);

	if (ctx->result) {
		LOG_ERR("%s link training failed (%d)", ctx->dev->name, ctx->result);
	} else {
		LOG_INF("%s link training done", ctx->dev->name);
	}

	/*
	 * Set DONE last: cmd_ltpi_init() treats DONE as "safe to recreate
	 * ctx->thread", and it also k_thread_join()s before doing so, but
	 * there is no reason to make that join wait any longer than it has to.
	 */
	atomic_set(&ctx->state, LTPI_SHELL_DONE);
}

static int cmd_ltpi_init(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	struct ltpi_shell_ctx *ctx;
	int timeout_ms = 0;
	int idx, err;

	dev = device_get_binding(argv[1]);
	if (!dev || !is_ltpi_device(dev)) {
		shell_error(sh, "LTPI: device %s not found", argv[1]);
		return -ENODEV;
	}

	if (argc > 2) {
		timeout_ms = (int)shell_strtol(argv[2], 0, &err);
		if (err || timeout_ms < 0) {
			shell_error(sh, "LTPI: invalid timeout_ms '%s'", argv[2]);
			return -EINVAL;
		}
	}

	ctx = ltpi_shell_ctx_find_or_alloc(dev);
	if (!ctx) {
		shell_error(sh, "LTPI: too many concurrent devices (max %d)",
			    LTPI_SHELL_MAX_DEV);
		return -ENOMEM;
	}

	if (atomic_get(&ctx->state) == LTPI_SHELL_RUNNING) {
		shell_warn(sh, "LTPI: %s link training already in progress", argv[1]);
		return -EBUSY;
	}

	/*
	 * ctx->thread may still be a few instructions away from actually
	 * exiting even though its state already flipped to DONE (or IDLE, on
	 * a fresh slot that was never started) -- join it before reusing the
	 * same k_thread object, rather than risk re-creating a still-live one.
	 */
	if (ctx->started) {
		k_thread_join(&ctx->thread, K_FOREVER);
	}

	ctx->timeout_ms = timeout_ms;
	atomic_set(&ctx->state, LTPI_SHELL_RUNNING);

	idx = ctx - ltpi_shell_ctxs;
	k_thread_create(&ctx->thread, ltpi_shell_stacks[idx],
			K_THREAD_STACK_SIZEOF(ltpi_shell_stacks[idx]),
			ltpi_link_thread_entry, ctx, NULL, NULL,
			LTPI_SHELL_THREAD_PRIO, 0, K_NO_WAIT);
	ctx->started = true;
	k_thread_name_set(&ctx->thread, "ltpi_link");

	shell_print(sh, "LTPI: %s link training started in background (timeout_ms=%d)",
		    argv[1], timeout_ms);
	shell_print(sh, "LTPI: check progress with 'ltpi status %s' or the log output",
		    argv[1]);

	return 0;
}

static int cmd_ltpi_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	struct ltpi_shell_ctx *ctx;
	struct ltpi_ctrl_status_regs *status;
	int ret;

	dev = device_get_binding(argv[1]);
	if (!dev || !is_ltpi_device(dev)) {
		shell_error(sh, "LTPI: device %s not found", argv[1]);
		return -ENODEV;
	}

	/* Read-only: never allocate a slot just for a status query. */
	ctx = ltpi_shell_ctx_find(dev);
	if (ctx) {
		switch (atomic_get(&ctx->state)) {
		case LTPI_SHELL_RUNNING:
			shell_print(sh, "LTPI: %s link training in progress", argv[1]);
			break;
		case LTPI_SHELL_DONE:
			shell_print(sh, "LTPI: %s last link training result: %s (%d)",
				    argv[1], ctx->result ? "failed" : "done", ctx->result);
			break;
		default:
			shell_print(sh, "LTPI: %s has not been started via 'ltpi init' yet",
				    argv[1]);
			break;
		}
	}

	ret = ltpi_get_status(dev, &status);
	if (ret) {
		shell_error(sh, "LTPI: %s get_status failed (%d)", argv[1], ret);
		return ret;
	}

	shell_print(sh, "LTPI: %s link_status=0x%08x", argv[1], status->link_status);

	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, LTPI_DEVICE_PREFIX);

	entry->syntax = (dev) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_ltpi_device_name, device_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ltpi_cmds,
			       SHELL_CMD_ARG(init, &dsub_ltpi_device_name,
					     "<device> [<timeout_ms>]\n"
					     "Start LTPI link training in the background.",
					     cmd_ltpi_init, 2, 1),
			       SHELL_CMD_ARG(status, &dsub_ltpi_device_name,
					     "<device>\n"
					     "Show link training progress/result.",
					     cmd_ltpi_status, 2, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ltpi, &sub_ltpi_cmds, "ASPEED LTPI commands", NULL);
