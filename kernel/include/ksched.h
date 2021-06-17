/*
 * Copyright (c) 2016-2017 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_KERNEL_INCLUDE_KSCHED_H_
#define ZEPHYR_KERNEL_INCLUDE_KSCHED_H_

#include <kernel_structs.h>
#include <kernel_internal.h>
#include <timeout_q.h>
#include <tracing/tracing.h>
#include <stdbool.h>

BUILD_ASSERT(K_LOWEST_APPLICATION_THREAD_PRIO
	     >= K_HIGHEST_APPLICATION_THREAD_PRIO);

#ifdef CONFIG_MULTITHREADING
#define Z_VALID_PRIO(prio, entry_point)				     \
	(((prio) == K_IDLE_PRIO && z_is_idle_thread_entry(entry_point)) || \
	 ((K_LOWEST_APPLICATION_THREAD_PRIO			     \
	   >= K_HIGHEST_APPLICATION_THREAD_PRIO)		     \
	  && (prio) >= K_HIGHEST_APPLICATION_THREAD_PRIO	     \
	  && (prio) <= K_LOWEST_APPLICATION_THREAD_PRIO))

#define Z_ASSERT_VALID_PRIO(prio, entry_point) do { \
	__ASSERT(Z_VALID_PRIO((prio), (entry_point)), \
		 "invalid priority (%d); allowed range: %d to %d", \
		 (prio), \
		 K_LOWEST_APPLICATION_THREAD_PRIO, \
		 K_HIGHEST_APPLICATION_THREAD_PRIO); \
	} while (false)
#else
#define Z_VALID_PRIO(prio, entry_point) ((prio) == -1)
#define Z_ASSERT_VALID_PRIO(prio, entry_point) __ASSERT((prio) == -1, "")
#endif

void z_sched_init(void);
void z_move_thread_to_end_of_prio_q(struct k_thread *thread);
int z_is_thread_time_slicing(struct k_thread *thread);
void z_unpend_thread_no_timeout(struct k_thread *thread);
struct k_thread *z_unpend1_no_timeout(_wait_q_t *wait_q);
int z_pend_curr(struct k_spinlock *lock, k_spinlock_key_t key,
	       _wait_q_t *wait_q, k_timeout_t timeout);
int z_pend_curr_irqlock(uint32_t key, _wait_q_t *wait_q, k_timeout_t timeout);
void z_pend_thread(struct k_thread *thread, _wait_q_t *wait_q,
		   k_timeout_t timeout);
void z_reschedule(struct k_spinlock *lock, k_spinlock_key_t key);
void z_reschedule_irqlock(uint32_t key);
struct k_thread *z_unpend_first_thread(_wait_q_t *wait_q);
void z_unpend_thread(struct k_thread *thread);
int z_unpend_all(_wait_q_t *wait_q);
void z_thread_priority_set(struct k_thread *thread, int prio);
bool z_set_prio(struct k_thread *thread, int prio);
void *z_get_next_switch_handle(void *interrupted);
void idle(void *unused1, void *unused2, void *unused3);
void z_time_slice(int ticks);
void z_reset_time_slice(void);
void z_sched_abort(struct k_thread *thread);
void z_sched_ipi(void);
void z_sched_start(struct k_thread *thread);
void z_ready_thread(struct k_thread *thread);
void z_requeue_current(struct k_thread *curr);
struct k_thread *z_swap_next_thread(void);
void z_thread_abort(struct k_thread *thread);

static inline void z_pend_curr_unlocked(_wait_q_t *wait_q, k_timeout_t timeout)
{
	(void) z_pend_curr_irqlock(arch_irq_lock(), wait_q, timeout);
}

static inline void z_reschedule_unlocked(void)
{
	(void) z_reschedule_irqlock(arch_irq_lock());
}

static inline bool z_is_idle_thread_entry(void *entry_point)
{
	return entry_point == idle;
}

static inline bool z_is_idle_thread_object(struct k_thread *thread)
{
#ifdef CONFIG_MULTITHREADING
#ifdef CONFIG_SMP
	return thread->base.is_idle;
#else
	return thread == &z_idle_threads[0];
#endif
#else
	return false;
#endif /* CONFIG_MULTITHREADING */
}

static inline bool z_is_thread_suspended(struct k_thread *thread)
{
	return (thread->base.thread_state & _THREAD_SUSPENDED) != 0U;
}

static inline bool z_is_thread_pending(struct k_thread *thread)
{
	return (thread->base.thread_state & _THREAD_PENDING) != 0U;
}

static inline bool z_is_thread_prevented_from_running(struct k_thread *thread)
{
	uint8_t state = thread->base.thread_state;

	return (state & (_THREAD_PENDING | _THREAD_PRESTART | _THREAD_DEAD |
			 _THREAD_DUMMY | _THREAD_SUSPENDED)) != 0U;

}

static inline bool z_is_thread_timeout_active(struct k_thread *thread)
{
	return !z_is_inactive_timeout(&thread->base.timeout);
}

static inline bool z_is_thread_ready(struct k_thread *thread)
{
	return !((z_is_thread_prevented_from_running(thread)) != 0U ||
		 z_is_thread_timeout_active(thread));
}

static inline bool z_has_thread_started(struct k_thread *thread)
{
	return (thread->base.thread_state & _THREAD_PRESTART) == 0U;
}

static inline bool z_is_thread_state_set(struct k_thread *thread, uint32_t state)
{
	return (thread->base.thread_state & state) != 0U;
}

static inline bool z_is_thread_queued(struct k_thread *thread)
{
	return z_is_thread_state_set(thread, _THREAD_QUEUED);
}

static inline void z_mark_thread_as_suspended(struct k_thread *thread)
{
	thread->base.thread_state |= _THREAD_SUSPENDED;

	SYS_PORT_TRACING_FUNC(k_thread, sched_suspend, thread);
}

static inline void z_mark_thread_as_not_suspended(struct k_thread *thread)
{
	thread->base.thread_state &= ~_THREAD_SUSPENDED;

	SYS_PORT_TRACING_FUNC(k_thread, sched_resume, thread);
}

static inline void z_mark_thread_as_started(struct k_thread *thread)
{
	thread->base.thread_state &= ~_THREAD_PRESTART;
}

static inline void z_mark_thread_as_pending(struct k_thread *thread)
{
	thread->base.thread_state |= _THREAD_PENDING;
}

static inline void z_mark_thread_as_not_pending(struct k_thread *thread)
{
	thread->base.thread_state &= ~_THREAD_PENDING;
}

static inline void z_set_thread_states(struct k_thread *thread, uint32_t states)
{
	thread->base.thread_state |= states;
}

static inline void z_reset_thread_states(struct k_thread *thread,
					uint32_t states)
{
	thread->base.thread_state &= ~states;
}

static inline bool z_is_under_prio_ceiling(int prio)
{
	return prio >= CONFIG_PRIORITY_CEILING;
}

static inline int z_get_new_prio_with_ceiling(int prio)
{
	return z_is_under_prio_ceiling(prio) ? prio : CONFIG_PRIORITY_CEILING;
}

static inline bool z_is_prio1_higher_than_or_equal_to_prio2(int prio1, int prio2)
{
	return prio1 <= prio2;
}

static inline bool z_is_prio_higher_or_equal(int prio1, int prio2)
{
	return z_is_prio1_higher_than_or_equal_to_prio2(prio1, prio2);
}

static inline bool z_is_prio1_lower_than_or_equal_to_prio2(int prio1, int prio2)
{
	return prio1 >= prio2;
}

static inline bool z_is_prio1_higher_than_prio2(int prio1, int prio2)
{
	return prio1 < prio2;
}

static inline bool z_is_prio_higher(int prio, int test_prio)
{
	return z_is_prio1_higher_than_prio2(prio, test_prio);
}

static inline bool z_is_prio_lower_or_equal(int prio1, int prio2)
{
	return z_is_prio1_lower_than_or_equal_to_prio2(prio1, prio2);
}

int32_t z_sched_prio_cmp(struct k_thread *thread_1, struct k_thread *thread_2);

static inline bool _is_valid_prio(int prio, void *entry_point)
{
	if (prio == K_IDLE_PRIO && z_is_idle_thread_entry(entry_point)) {
		return true;
	}

	if (!z_is_prio_higher_or_equal(prio,
				       K_LOWEST_APPLICATION_THREAD_PRIO)) {
		return false;
	}

	if (!z_is_prio_lower_or_equal(prio,
				      K_HIGHEST_APPLICATION_THREAD_PRIO)) {
		return false;
	}

	return true;
}

static inline void _ready_one_thread(_wait_q_t *wq)
{
	struct k_thread *thread = z_unpend_first_thread(wq);

	if (thread != NULL) {
		z_ready_thread(thread);
	}
}

static inline void z_sched_lock(void)
{
	__ASSERT(!arch_is_in_isr(), "");
	__ASSERT(_current->base.sched_locked != 1U, "");

	--_current->base.sched_locked;

	compiler_barrier();
}

static ALWAYS_INLINE void z_sched_unlock_no_reschedule(void)
{
	__ASSERT(!arch_is_in_isr(), "");
	__ASSERT(_current->base.sched_locked != 0U, "");

	compiler_barrier();

	++_current->base.sched_locked;
}

static ALWAYS_INLINE bool z_is_thread_timeout_expired(struct k_thread *thread)
{
#ifdef CONFIG_SYS_CLOCK_EXISTS
	return thread->base.timeout.dticks == _EXPIRED;
#else
	return 0;
#endif
}

/*
 * APIs for working with the Zephyr kernel scheduler. Intended for use in
 * management of IPC objects, either in the core kernel or other IPC
 * implemented by OS compatibility layers, providing basic wait/wake operations
 * with spinlocks used for synchronization.
 *
 * These APIs are public and will be treated as contract, even if the
 * underlying scheduler implementation changes.
 */

/**
 * Wake up a thread pending on the provided wait queue
 *
 * Given a wait_q, wake up the highest priority thread on the queue. If the
 * queue was empty just return false.
 *
 * Otherwise, do the following, in order,  holding sched_spinlock the entire
 * time so that the thread state is guaranteed not to change:
 * - Set the thread's swap return values to swap_retval and swap_data
 * - un-pend and ready the thread, but do not invoke the scheduler.
 *
 * Repeated calls to this function until it returns false is a suitable
 * way to wake all threads on the queue.
 *
 * It is up to the caller to implement locking such that the return value of
 * this function (whether a thread was woken up or not) does not immediately
 * become stale. Calls to wait and wake on the same wait_q object must have
 * synchronization. Calling this without holding any spinlock is a sign that
 * this API is not being used properly.
 *
 * @param wait_q Wait queue to wake up the highest prio thread
 * @param swap_retval Swap return value for woken thread
 * @param swap_data Data return value to supplement swap_retval. May be NULL.
 * @retval true If a thread was woken up
 * @retval false If the wait_q was empty
 */
bool z_sched_wake(_wait_q_t *wait_q, int swap_retval, void *swap_data);

/**
 * Wake up all threads pending on the provided wait queue
 *
 * Convenience function to invoke z_sched_wake() on all threads in the queue
 * until there are no more to wake up.
 *
 * @param wait_q Wait queue to wake up the highest prio thread
 * @param swap_retval Swap return value for woken thread
 * @param swap_data Data return value to supplement swap_retval. May be NULL.
 * @retval true If any threads were woken up
 * @retval false If the wait_q was empty
 */
static inline bool z_sched_wake_all(_wait_q_t *wait_q, int swap_retval,
				    void *swap_data)
{
	bool woken = false;

	while (z_sched_wake(wait_q, swap_retval, swap_data)) {
		woken = true;
	}

	/* True if we woke at least one thread up */
	return woken;
}

/**
 * Atomically put the current thread to sleep on a wait queue, with timeout
 *
 * The thread will be added to the provided waitqueue. The lock, which should
 * be held by the caller with the provided key, will be released once this is
 * completely done and we have swapped out.
 *
 * The return value and data pointer is set by whoever woke us up via
 * z_sched_wake.
 *
 * @param lock Address of spinlock to release when we swap out
 * @param key Key to the provided spinlock when it was locked
 * @param wait_q Wait queue to go to sleep on
 * @param timeout Waiting period to be woken up, or K_FOREVER to wait
 *                indefinitely.
 * @param data Storage location for data pointer set when thread was woken up.
 *             May be NULL if not used.
 * @retval Return value set by whatever woke us up, or -EAGAIN if the timeout
 *         expired without being woken up.
 */
int z_sched_wait(struct k_spinlock *lock, k_spinlock_key_t key,
		 _wait_q_t *wait_q, k_timeout_t timeout, void **data);

#endif /* ZEPHYR_KERNEL_INCLUDE_KSCHED_H_ */
