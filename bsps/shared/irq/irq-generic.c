/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup bsp_interrupt
 *
 * @brief This source file contains the generic interrupt controller support
 *   implementation.
 */

/*
 * Copyright (C) 2008, 2018 embedded brains GmbH (http://www.embedded-brains.de)
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

#include <bsp/irq-generic.h>
#include <bsp/fatal.h>

#include <stdlib.h>

#include <rtems/malloc.h>

#ifdef BSP_INTERRUPT_USE_INDEX_TABLE
  bsp_interrupt_handler_index_type bsp_interrupt_handler_index_table
    [BSP_INTERRUPT_VECTOR_COUNT];
#endif

rtems_interrupt_entry bsp_interrupt_handler_table
  [BSP_INTERRUPT_HANDLER_TABLE_SIZE];

/* The last entry indicates if everything is initialized */
uint8_t bsp_interrupt_handler_unique_table
  [ ( BSP_INTERRUPT_HANDLER_TABLE_SIZE + 7 + 1 ) / 8 ];

void bsp_interrupt_handler_empty( void *arg )
{
  rtems_vector_number vector = (rtems_vector_number) (uintptr_t) arg;

  bsp_interrupt_handler_default( vector );
}

#ifdef RTEMS_SMP
  static void bsp_interrupt_handler_do_nothing(void *arg)
  {
    (void) arg;
  }
#endif

static inline void bsp_interrupt_set_handler_unique(
  rtems_vector_number index,
  bool unique
)
{
  rtems_vector_number i = index / 8;
  rtems_vector_number s = index % 8;
  if (unique) {
    bsp_interrupt_handler_unique_table [i] |= (uint8_t) (0x1U << s);
  } else {
    bsp_interrupt_handler_unique_table [i] &= (uint8_t) ~(0x1U << s);
  }
}

static inline void bsp_interrupt_set_initialized(void)
{
  bsp_interrupt_set_handler_unique(BSP_INTERRUPT_HANDLER_TABLE_SIZE, true);
}

static inline void bsp_interrupt_clear_handler_entry(
  rtems_interrupt_entry *e,
  rtems_vector_number vector
)
{
  e->handler = bsp_interrupt_handler_empty;
  bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
  e->arg = (void *) (uintptr_t) vector;
  e->info = NULL;
  e->next = NULL;
}

static inline bool bsp_interrupt_allocate_handler_index(
  rtems_vector_number vector,
  rtems_vector_number *index
)
{
  #ifdef BSP_INTERRUPT_USE_INDEX_TABLE
    rtems_vector_number i = 0;

    /* The first entry will remain empty */
    for (i = 1; i < BSP_INTERRUPT_HANDLER_TABLE_SIZE; ++i) {
      const rtems_interrupt_entry *e = &bsp_interrupt_handler_table [i];
      if (bsp_interrupt_is_empty_handler_entry(e)) {
        *index = i;
        return true;
      }
    }

    return false;
  #else
    *index = bsp_interrupt_handler_index(vector);
    return true;
  #endif
}

rtems_status_code bsp_interrupt_check_and_lock(
  rtems_vector_number     vector,
  rtems_interrupt_handler handler
)
{
  if ( !bsp_interrupt_is_initialized() ) {
    return RTEMS_INCORRECT_STATE;
  }

  if ( handler == NULL ) {
    return RTEMS_INVALID_ADDRESS;
  }

  if ( !bsp_interrupt_is_valid_vector( vector ) ) {
    return RTEMS_INVALID_ID;
  }

  if ( rtems_interrupt_is_in_progress() ) {
    return RTEMS_CALLED_FROM_ISR;
  }

  bsp_interrupt_lock();

  return RTEMS_SUCCESSFUL;
}

void bsp_interrupt_initialize(void)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  size_t i = 0;

  /* Initialize handler table */
  for (i = 0; i < BSP_INTERRUPT_HANDLER_TABLE_SIZE; ++i) {
    bsp_interrupt_handler_table [i].handler = bsp_interrupt_handler_empty;
    bsp_interrupt_handler_table [i].arg = (void *) i;
  }

  sc = bsp_interrupt_facility_initialize();
  if (sc != RTEMS_SUCCESSFUL) {
    bsp_fatal(BSP_FATAL_INTERRUPT_INITIALIZATION);
  }

  bsp_interrupt_set_initialized();
}

/**
 * @brief Installs an interrupt handler.
 *
 * @ingroup bsp_interrupt
 *
 * @return In addition to the standard status codes this function returns:
 * - If the BSP interrupt support is not initialized RTEMS_INTERNAL_ERROR will
 * be returned.
 * - If not enough memory for a new handler is available RTEMS_NO_MEMORY will
 * be returned
 *
 * @see rtems_interrupt_handler_install()
 */
static rtems_status_code bsp_interrupt_handler_install(
  rtems_vector_number vector,
  const char *info,
  rtems_option options,
  rtems_interrupt_handler handler,
  void *arg
)
{
  rtems_status_code sc;
  rtems_interrupt_level level;
  rtems_vector_number index = 0;
  rtems_interrupt_entry *head = NULL;
  bool enable_vector = false;
  bool replace = RTEMS_INTERRUPT_IS_REPLACE(options);

  sc = bsp_interrupt_check_and_lock( vector, handler );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  /* Get handler table index */
  index = bsp_interrupt_handler_index(vector);

  /* Get head entry of the handler list for current vector */
  head = &bsp_interrupt_handler_table [index];

  if (bsp_interrupt_is_empty_handler_entry(head)) {
    if (replace) {
      /* No handler to replace exists */
      bsp_interrupt_unlock();
      return RTEMS_UNSATISFIED;
    }

    /*
     * No real handler installed yet.  So allocate a new index in
     * the handler table and fill the entry with life.
     */
    if (bsp_interrupt_allocate_handler_index(vector, &index)) {
      bsp_interrupt_disable(level);
      bsp_interrupt_handler_table [index].arg = arg;
      bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
      bsp_interrupt_handler_table [index].handler = handler;
      #ifdef BSP_INTERRUPT_USE_INDEX_TABLE
        bsp_interrupt_handler_index_table [vector] = index;
      #endif
      bsp_interrupt_enable(level);
      bsp_interrupt_handler_table [index].info = info;
    } else {
      /* Handler table is full */
      bsp_interrupt_unlock();
      return RTEMS_NO_MEMORY;
    }

    /* This is the first handler so enable the vector later */
    enable_vector = true;
  } else {
    rtems_interrupt_entry *current = head;
    rtems_interrupt_entry *tail = NULL;
    rtems_interrupt_entry *match = NULL;

    /* Ensure that a unique handler remains unique */
    if (
      !replace
        && (RTEMS_INTERRUPT_IS_UNIQUE(options)
          || bsp_interrupt_is_handler_unique(index))
    ) {
      /*
       * Tried to install a unique handler on a not empty
       * list or there is already a unique handler installed.
       */
      bsp_interrupt_unlock();
      return RTEMS_RESOURCE_IN_USE;
    }

    /*
     * Search for the list tail and check if the handler is already
     * installed.
     */
    do {
      if (
        match == NULL
          && (current->handler == handler || replace)
          && current->arg == arg
      ) {
        match = current;
      }
      tail = current;
      current = current->next;
    } while (current != NULL);

    if (replace) {
      /* Ensure that a handler to replace exists */
      if (match == NULL) {
        bsp_interrupt_unlock();
        return RTEMS_UNSATISFIED;
      }

      /* Use existing entry */
      current = match;
    } else {
      /* Ensure the handler is not already installed */
      if (match != NULL) {
        /* The handler is already installed */
        bsp_interrupt_unlock();
        return RTEMS_TOO_MANY;
      }

      /* Allocate a new entry */
      current = rtems_malloc(sizeof(*current));
      if (current == NULL) {
        /* Not enough memory */
        bsp_interrupt_unlock();
        return RTEMS_NO_MEMORY;
      }
    }

    /* Update existing entry or set new entry */
    current->handler = handler;
    current->info = info;

    if (!replace) {
      /* Set new entry */
      current->arg = arg;
      current->next = NULL;

      /* Link to list tail */
      bsp_interrupt_disable(level);
      bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
      tail->next = current;
      bsp_interrupt_enable(level);
    }
  }

  /* Make the handler unique if necessary */
  bsp_interrupt_set_handler_unique(index, RTEMS_INTERRUPT_IS_UNIQUE(options));

  /* Enable the vector if necessary */
  if (enable_vector) {
    bsp_interrupt_vector_enable(vector);
  }

  /* Unlock */
  bsp_interrupt_unlock();

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Removes an interrupt handler.
 *
 * @ingroup bsp_interrupt
 *
 * @return In addition to the standard status codes this function returns
 * RTEMS_INTERNAL_ERROR if the BSP interrupt support is not initialized.
 *
 * @see rtems_interrupt_handler_remove().
 */
static rtems_status_code bsp_interrupt_handler_remove(
  rtems_vector_number vector,
  rtems_interrupt_handler handler,
  void *arg
)
{
  rtems_status_code sc;
  rtems_interrupt_level level;
  rtems_vector_number index = 0;
  rtems_interrupt_entry *head = NULL;
  rtems_interrupt_entry *current = NULL;
  rtems_interrupt_entry *previous = NULL;
  rtems_interrupt_entry *match = NULL;

  sc = bsp_interrupt_check_and_lock( vector, handler );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  /* Get handler table index */
  index = bsp_interrupt_handler_index(vector);

  /* Get head entry of the handler list for current vector */
  head = &bsp_interrupt_handler_table [index];

  /* Search for a matching entry */
  current = head;
  do {
    if (current->handler == handler && current->arg == arg) {
      match = current;
      break;
    }
    previous = current;
    current = current->next;
  } while (current != NULL);

  /* Remove the matching entry */
  if (match != NULL) {
    if (match->next != NULL) {
      /*
       * The match has a successor.  A successor is always
       * allocated.  So replace the match with its successor
       * and free the successor entry.
       */
      current = match->next;

      bsp_interrupt_disable(level);
      #ifdef RTEMS_SMP
        match->handler = bsp_interrupt_handler_do_nothing;
        bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
      #endif
      match->arg = current->arg;
      bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
      match->handler = current->handler;
      match->info = current->info;
      match->next = current->next;
      bsp_interrupt_enable(level);

      free(current);
    } else if (match == head) {
      /*
       * The match is the list head and has no successor.
       * The list head is stored in a static table so clear
       * this entry.  Since now the list is empty disable the
       * vector.
       */

      /* Disable the vector */
      bsp_interrupt_vector_disable(vector);

      /* Clear entry */
      bsp_interrupt_disable(level);
      bsp_interrupt_clear_handler_entry(head, vector);
      #ifdef BSP_INTERRUPT_USE_INDEX_TABLE
        bsp_interrupt_handler_index_table [vector] = 0;
      #endif
      bsp_interrupt_enable(level);

      /* Allow shared handlers */
      bsp_interrupt_set_handler_unique(index, false);
    } else {
      /*
       * The match is the list tail and has a predecessor.
       * So terminate the predecessor and free the match.
       */
      bsp_interrupt_disable(level);
      previous->next = NULL;
      bsp_interrupt_fence(ATOMIC_ORDER_RELEASE);
      bsp_interrupt_enable(level);

      free(match);
    }
  } else {
    /* No matching entry found */
    bsp_interrupt_unlock();
    return RTEMS_UNSATISFIED;
  }

  /* Unlock */
  bsp_interrupt_unlock();

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_interrupt_handler_install(
  rtems_vector_number vector,
  const char *info,
  rtems_option options,
  rtems_interrupt_handler handler,
  void *arg
)
{
  return bsp_interrupt_handler_install(vector, info, options, handler, arg);
}

rtems_status_code rtems_interrupt_handler_remove(
  rtems_vector_number vector,
  rtems_interrupt_handler handler,
  void *arg
)
{
  return bsp_interrupt_handler_remove(vector, handler, arg);
}

bool bsp_interrupt_handler_is_empty(rtems_vector_number vector)
{
  rtems_vector_number index = 0;
  rtems_interrupt_entry *head = NULL;
  bool empty;

  /* For use in interrupts so no lock. */

  /* Get handler table index */
  index = bsp_interrupt_handler_index(vector);

  /* Get head entry of the handler list for the vector */
  head = &bsp_interrupt_handler_table [index];

  empty = bsp_interrupt_is_empty_handler_entry(head);

  return empty;
}
