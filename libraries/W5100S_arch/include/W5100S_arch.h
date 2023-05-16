/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _W5100S_ARCH_H
#define _W5100S_ARCH_H

#include "pico.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "W5100S_driver.h"
#include "pico/async_context.h"

#ifdef W5100S_ARCH_HEADER
#include __XSTRING(W5100S_ARCH_HEADER)
#else
#if W5100S_ARCH_POLL
#include "W5100S_arch/arch_poll.h"
#elif W5100S_ARCH_THREADSAFE_BACKGROUND
#include "W5100S_arch/arch_threadsafe_background.h"
#elif W5100S_ARCH_FREERTOS
#include "W5100S_arch/arch_freertos.h"
#else
#error must specify support W5100S_arch architecture type or set W5100S_ARCH_HEADER
#endif
#endif

/**
 * \defgroup W5100S_driver W5100S_driver
 * \ingroup pico_W5100S_arch
 * \brief Driver used for Wiznet W5100S wired ethernet
*/

/** \file pico/W5100S_arch.h
 *  \defgroup pico_W5100S_arch pico_W5100S_arch
 *
 * Architecture for integrating the W5100S driver (for Wiznet W5100S wired ethernet support on Pico) and lwIP (for TCP/IP stack) into the SDK.
 *
 * Both the low level \c W5100S_driver and the lwIP stack require periodic servicing, and have limitations
 * on whether they can be called from multiple cores/threads.
 *
 * \c pico_W5100S_arch attempts to abstract these complications into several behavioral groups:
 *
 * * \em 'poll' - This not multi-core/IRQ safe, and requires the user to call \ref W5100S_arch_poll periodically from their main loop
 * * \em 'thread_safe_background' - This is multi-core/thread/task safe, and maintenance of the driver and TCP/IP stack is handled automatically in the background
 * * \em 'freertos' - This is multi-core/thread/task safe, and uses a separate FreeRTOS task to handle lwIP and and driver work.
 *
 * As of right now, lwIP is the only supported TCP/IP stack, however the use of \c pico_W5100S_arch is intended to be independent of
 * the particular TCP/IP stack used (and possibly Bluetooth stack used) in the future. For this reason, the integration of lwIP
 * is handled in the base (\c pico_W5100S_arch) library based on the #define \ref W5100S_LWIP used by the \c W5100S_driver.
 *
 * \note As of version 1.5.0 of the Raspberry Pi Pico SDK, the \c pico_W5100S_arch library no longer directly implements
 * the distinct behavioral abstractions. This is now handled by the more general \ref pico_async_context library. The
 * user facing behavior of pico_W5100S_arch has not changed as a result of this implementation detail, however pico_W5100S_arch
 * is now just a thin wrapper which creates an appropriate async_context and makes a simple call to add lwIP or W5100S_driver support
 * as appropriate. You are free to perform this context creation and adding of lwIP, W5100S_driver or indeed any other additional
 * future protocol/driver support to your async_context, however for now pico_W5100S_arch does still provide a few W5100S_ specific (i.e. Pico W)
 * APIs for connection management, locking and GPIO interaction.
 *
 * \note The connection management APIs at least may be moved
 * to a more generic library in a future release. The locking methods are now backed by their \ref pico_async_context equivalents, and
 * those methods may be used interchangeably (see \ref W5100S_arch_lwip_begin, \ref W5100S_arch_lwip_end and \ref W5100S_arch_lwip_check for more details).
 *
 * \note For examples of creating of your own async_context and addition of \c W5100S_driver and \c lwIP support, please
 * refer to the specific source files \c W5100S_arch_poll.c, \c W5100S_arch_threadsafe_background.c and \c W5100S_arch_freertos.c.
 *
 * Whilst you can use the \c pico_W5100S_arch library directly and specify \ref W5100S_LWIP (and other defines) yourself, several
 * other libraries are made available to the build which aggregate the defines and other dependencies for you:
 *
 * * \b pico_W5100S_arch_lwip_poll - For using the RAW lwIP API (in `NO_SYS=1` mode) without any background processing or multi-core/thread safety.
 *
 *    The user must call \ref pico_W5100S_poll periodically from their main loop.
 *
 *    This wrapper library:
 *    - Sets \c W5100S_LWIP=1 to enable lwIP support in \c pico_W5100S_arch and \c W5100S_driver.
 *    - Sets \c PICO_W5100S_ARCH_POLL=1 to select the polling behavior.
 *    - Adds the \c pico_lwip as a dependency to pull in lwIP.
 *
 * * \b pico_W5100S_arch_lwip_threadsafe_background - For using the RAW lwIP API (in `NO_SYS=1` mode) with multi-core/thread safety, and automatic servicing of the \c W5100S_driver and
 * lwIP in background.
 *
 *    Calls into the \c W5100S_driver high level API (W5100S.h) may be made from either core or from lwIP callbacks, however calls into lwIP (which
 * is not thread-safe) other than those made from lwIP callbacks, must be bracketed with \ref W5100S_arch_lwip_begin and \ref W5100S_arch_lwip_end. It is fine to bracket
 * calls made from within lwIP callbacks too; you just don't have to.
 *
 *    \note lwIP callbacks happen in a (low priority) IRQ context (similar to an alarm callback), so care should be taken when interacting
 *    with other code.
 *
 *    This wrapper library:
 *    - Sets \c W5100S_LWIP=1 to enable lwIP support in \c pico_W5100S_arch and \c W5100S_driver
 *    - Sets \c PICO_W5100S_ARCH_THREADSAFE_BACKGROUND=1 to select the thread-safe/non-polling behavior.
 *    - Adds the pico_lwip as a dependency to pull in lwIP.
 *
 *
 *    This library \em can also be used under the RP2040 port of FreeRTOS with lwIP in `NO_SYS=1` mode (allowing you to call \c W5100S_driver APIs
 * from any task, and to call lwIP from lwIP callbacks, or from any task if you bracket the calls with \ref W5100S_arch_lwip_begin and \ref W5100S_arch_lwip_end. Again, you should be
 * careful about what you do in lwIP callbacks, as you cannot call most FreeRTOS APIs from within an IRQ context. Unless you have good reason, you should probably
 * use the full FreeRTOS integration (with `NO_SYS=0`) provided by \c pico_W5100S_arch_lwip_sys_freertos.
 *
 * * \b pico_W5100S_arch_lwip_sys_freertos - For using the full lwIP API including blocking sockets in OS (`NO_SYS=0`) mode, along with with multi-core/task/thread safety, and automatic servicing of the \c W5100S_driver and
 * the lwIP stack.
 *
 *    This wrapper library:
 *    - Sets \c W5100S_LWIP=1 to enable lwIP support in \c pico_W5100S_arch and \c W5100S_driver.
 *    - Sets \c PICO_W5100S_ARCH_FREERTOS=1 to select the NO_SYS=0 lwip/FreeRTOS integration
 *    - Sets \c LWIP_PROVIDE_ERRNO=1 to provide error numbers needed for compilation without an OS
 *    - Adds the \c pico_lwip as a dependency to pull in lwIP.
 *    - Adds the lwIP/FreeRTOS code from lwip-contrib (in the contrib directory of lwIP)
 *
 *    Calls into the \c W5100S_driver high level API (W5100S.h) may be made from any task or from lwIP callbacks, but not from IRQs. Calls into the lwIP RAW API (which is not thread safe)
 *    must be bracketed with \ref W5100S_arch_lwip_begin and \ref W5100S_arch_lwip_end. It is fine to bracket calls made from within lwIP callbacks too; you just don't have to.
 *
 *    \note this wrapper library requires you to link FreeRTOS functionality with your application yourself.
 *
 * * \b pico_W5100S_arch_none - If you do not need the TCP/IP stack but wish to use the on-board LED.
 *
 *    This wrapper library:
 *    - Sets \c W5100S_LWIP=0 to disable lwIP support in \c pico_W5100S_arch and \c W5100S_driver
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_W5100S_ARCH, Enable/disable assertions in the pico_W5100S_arch module, type=bool, default=0, group=pico_W5100S_arch
#ifndef PARAM_ASSERTIONS_ENABLED_W5100S_ARCH
#define PARAM_ASSERTIONS_ENABLED_W5100S_ARCH 0
#endif

// PICO_CONFIG: PICO_W5100S_ARCH_DEBUG_ENABLED, Enable/disable some debugging output in the pico_W5100S_arch module, type=bool, default=1 in debug builds, group=pico_W5100S_arch
#ifndef PICO_W5100S_ARCH_DEBUG_ENABLED
#ifndef NDEBUG
#define PICO_W5100S_ARCH_DEBUG_ENABLED 1
#else
#define PICO_W5100S_ARCH_DEBUG_ENABLED 0
#endif
#endif

/*!
 * \brief Initialize the W5100S architecture
 * \ingroup pico_W5100S_arch
 *
 * This method initializes the `W5100S_driver` code and initializes the lwIP stack (if it
 * was enabled at build time). This method must be called prior to using any other \c pico_W5100S_arch,
 * \c W5100S_driver or lwIP functions.
 *
 * By default this method initializes the W5100S_arch code's own async_context by calling
 * \ref W5100S_arch_init_default_async_context, however the user can specify use of their own async_context
 * by calling \ref W5100S_arch_set_async_context() before calling this method
 *
 * \return 0 if the initialization is successful, an error code otherwise \see pico_error_codes
 */
int W5100S_arch_init(void);

/*!
 * \brief De-initialize the W5100S architecture
 * \ingroup pico_W5100S_arch
 *
 * This method de-initializes the `W5100S_driver` code and de-initializes the lwIP stack (if it
 * was enabled at build time). Note this method should always be called from the same core (or RTOS
 * task, depending on the environment) as \ref W5100S_arch_init.
 *
 * Additionally if the W5100S_arch is using its own async_context instance, then that instance is de-initialized.
 */
void W5100S_arch_deinit(void);

/*!
 * \brief Return the current async_context currently in use by the W5100S_arch code
 * \ingroup pico_W5100S_arch
 *
 * \return the async_context.
 */
async_context_t *W5100S_arch_async_context(void);

/*!
 * \brief Set the async_context to be used by the W5100S_arch_init
 * \ingroup pico_W5100S_arch
 *
 * \note This method must be called before calling W5100S_arch_init 
 * if you wish to use a custom async_context instance.
 *
 * \param context the async_context to be used
 */
void W5100S_arch_set_async_context(async_context_t *context);

/*!
 * \brief Initialize the default async_context for the current W5100S_arch type
 * \ingroup pico_W5100S_arch
 *
 * This method initializes and returns a pointer to the static async_context associated
 * with W5100S_arch. This method is called by \ref W5100S_arch_init automatically
 * if a different async_context has not been set by \ref W5100S_arch_set_async_context
 *
 * \return the context or NULL if initialization failed.
 */
async_context_t *W5100S_arch_init_default_async_context(void);

/*!
 * \brief Perform any processing required by the \c W5100S_driver or the TCP/IP stack
 * \ingroup pico_W5100S_arch
 *
 * This method must be called periodically from the main loop when using a
 * \em polling style \c pico_W5100S_arch (e.g. \c pico_W5100S_arch_lwip_poll ). It
 * may be called in other styles, but it is unnecessary to do so.
 */
void W5100S_arch_poll(void);

/*!
 * \brief Sleep until there is W5100S_driver work to be done
 * \ingroup pico_W5100S_arch
 *
 * This method may be called by code that is waiting for an event to
 * come from the W5100S_driver, and has no work to do, but would like
 * to sleep without blocking any background work associated with the W5100S_driver.
 *
 * \param until the time to wait until if there is no work to do.
 */
void W5100S_arch_wait_for_work_until(absolute_time_t until);

/*!
 * \fn W5100S_arch_lwip_begin
 * \brief Acquire any locks required to call into lwIP
 * \ingroup pico_W5100S_arch
 *
 * The lwIP API is not thread safe. You should surround calls into the lwIP API
 * with calls to this method and \ref W5100S_arch_lwip_end. Note these calls are not
 * necessary (but harmless) when you are calling back into the lwIP API from an lwIP callback.
 * If you are using single-core polling only (pico_W5100S_arch_poll) then these calls are no-ops
 * anyway it is good practice to call them anyway where they are necessary.
 *
 * \note as of SDK release 1.5.0, this is now equivalent to calling \ref pico_async_context_acquire_lock_blocking
 * on the async_context associated with W5100S_arch and lwIP.
 *
 * \sa W5100S_arch_lwip_end
 * \sa W5100S_arch_lwip_protect
 * \sa async_context_acquire_lock_blocking
 * \sa W5100S_arch_async_context
 */
static inline void W5100S_arch_lwip_begin(void) {
    W5100S_thread_enter();
}

/*!
 * \fn void W5100S_arch_lwip_end(void)
 * \brief Release any locks required for calling into lwIP
 * \ingroup pico_W5100S_arch
 *
 * The lwIP API is not thread safe. You should surround calls into the lwIP API
 * with calls to \ref W5100S_arch_lwip_begin and this method. Note these calls are not
 * necessary (but harmless) when you are calling back into the lwIP API from an lwIP callback.
 * If you are using single-core polling only (pico_W5100S_arch_poll) then these calls are no-ops
 * anyway it is good practice to call them anyway where they are necessary.
 *
 * \note as of SDK release 1.5.0, this is now equivalent to calling \ref async_context_release_lock
 * on the async_context associated with W5100S_arch and lwIP.
 *
 * \sa W5100S_arch_lwip_begin
 * \sa W5100S_arch_lwip_protect
 * \sa async_context_release_lock
 * \sa W5100S_arch_async_context
 */
static inline void W5100S_arch_lwip_end(void) {
    W5100S_thread_exit();
}

/*!
 * \fn int W5100S_arch_lwip_protect(int (*func)(void *param), void *param)
 * \brief sad Release any locks required for calling into lwIP
 * \ingroup pico_W5100S_arch
 *
 * The lwIP API is not thread safe. You can use this method to wrap a function
 * with any locking required to call into the lwIP API. If you are using
 * single-core polling only (pico_W5100S_arch_poll) then there are no
 * locks to required, but it is still good practice to use this function.
 *
 * \param func the function ta call with any required locks held
 * \param param parameter to pass to \c func
 * \return the return value from \c func
 * \sa W5100S_arch_lwip_begin
 * \sa W5100S_arch_lwip_end
 */
static inline int W5100S_arch_lwip_protect(int (*func)(void *param), void *param) {
    W5100S_arch_lwip_begin();
    int rc = func(param);
    W5100S_arch_lwip_end();
    return rc;
}

/*!
 * \fn void W5100S_arch_lwip_check(void)
 * \brief Checks the caller has any locks required for calling into lwIP
 * \ingroup pico_W5100S_arch
 *
 * The lwIP API is not thread safe. You should surround calls into the lwIP API
 * with calls to \ref W5100S_arch_lwip_begin and this method. Note these calls are not
 * necessary (but harmless) when you are calling back into the lwIP API from an lwIP callback.
 *
 * This method will assert in debug mode, if the above conditions are not met (i.e. it is not safe to
 * call into the lwIP API)
 *
 * \note as of SDK release 1.5.0, this is now equivalent to calling \ref async_context_lock_check
 * on the async_context associated with W5100S_arch and lwIP.
 *
 * \sa W5100S_arch_lwip_begin
 * \sa W5100S_arch_lwip_protect
 * \sa async_context_lock_check
 * \sa W5100S_arch_async_context
 */

#ifdef __cplusplus
}
#endif

#endif
