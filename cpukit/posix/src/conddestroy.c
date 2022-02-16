/* SPDX-License-Identifier: BSD-2-Clause */

/**
 *  @file
 *
 *  @brief Destroy a Condition Variable
 *  @ingroup POSIXAPI
 */

/*
 *  COPYRIGHT (c) 1989-2007.
 *  On-Line Applications Research Corporation (OAR).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/posix/condimpl.h>

/**
 *  11.4.2 Initializing and Destroying a Condition Variable,
 *         P1003.1c/Draft 10, p. 87
 */
int pthread_cond_destroy( pthread_cond_t *cond )
{
  POSIX_Condition_variables_Control *the_cond;
  unsigned long                      flags;
  Thread_queue_Context               queue_context;

  the_cond = _POSIX_Condition_variables_Get( cond );
  POSIX_CONDITION_VARIABLES_VALIDATE_OBJECT( the_cond, flags );

  _Thread_queue_Context_initialize( &queue_context );
  _POSIX_Condition_variables_Acquire( the_cond, &queue_context );

  if ( !_Thread_queue_Is_empty( &the_cond->Queue.Queue ) ) {
    _POSIX_Condition_variables_Release( the_cond, &queue_context );
    return EBUSY;
  }

  _POSIX_Condition_variables_Release( the_cond, &queue_context );
  _POSIX_Condition_variables_Destroy( the_cond );
  return 0;
}
