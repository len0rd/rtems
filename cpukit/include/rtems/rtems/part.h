/**
 * @file
 *
 * @ingroup ClassicPart
 *
 * @brief Classic Partition Manager API
 */

/* COPYRIGHT (c) 1989-2008.
 * On-Line Applications Research Corporation (OAR).
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_RTEMS_PART_H
#define _RTEMS_RTEMS_PART_H

#include <rtems/rtems/attr.h>
#include <rtems/rtems/status.h>
#include <rtems/rtems/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @defgroup ClassicPart Partitions
 *
 *  @ingroup RTEMSAPIClassic
 *
 *  This encapsulates functionality related to the
 *  Classic API Partition Manager.
 */
/**@{*/

/**
 * @brief This constant defines the minimum alignment of a partition buffer in
 *   bytes.
 *
 * Use it with RTEMS_ALIGNED() to define the alignment of partition buffer
 * types or statically allocated partition buffer areas.
 */
#define RTEMS_PARTITION_ALIGNMENT CPU_SIZEOF_POINTER

/**
 *  @brief RTEMS Partition Create
 *
 *  Partition Manager
 *
 *  This routine implements the rtems_partition_create directive.  The
 *  partition will have the name name.  The memory area managed by
 *  the partition is of length bytes and starts at starting_address.
 *  The memory area will be divided into as many buffers of
 *  buffer_size bytes as possible.   The attribute_set determines if
 *  the partition is global or local.  It returns the id of the
 *  created partition in ID.
 */
rtems_status_code rtems_partition_create(
  rtems_name       name,
  void            *starting_address,
  uintptr_t        length,
  size_t           buffer_size,
  rtems_attribute  attribute_set,
  rtems_id        *id
);

/**
 * @brief RTEMS Partition Ident
 *
 * This routine implements the rtems_partition_ident directive.
 * This directive returns the partition ID associated with name.
 * If more than one partition is named name, then the partition
 * to which the ID belongs is arbitrary. node indicates the
 * extent of the search for the ID of the partition named name.
 * The search can be limited to a particular node or allowed to
 * encompass all nodes.
 *
 * @param[in] name is the user defined partition name
 * @param[in] node is(are) the node(s) to be searched
 * @param[in] id is the pointer to partition id
 *
 * @retval RTEMS_SUCCESSFUL if successful or error code if unsuccessful and
 * 		*id filled in with the partition id
 */
rtems_status_code rtems_partition_ident(
  rtems_name  name,
  uint32_t    node,
  rtems_id   *id
);

/**
 * @brief RTEMS Delete Partition
 *
 * This routine implements the rtems_partition_delete directive. The
 * partition indicated by ID is deleted, provided that none of its buffers
 * are still allocated.
 *
 * @param[in] id is the partition id
 *
 * @retval This method returns RTEMS_SUCCESSFUL if there was not an
 *         error. Otherwise, a status code is returned indicating the
 *         source of the error.
 */
rtems_status_code rtems_partition_delete(
  rtems_id id
);

/**
 * @brief RTEMS Get Partition Buffer
 *
 * This routine implements the rtems_partition_get_buffer directive. It
 * attempts to allocate a buffer from the partition associated with ID.
 * If a buffer is allocated, its address is returned in buffer.
 *
 * @param[in] id is the partition id
 * @param[out] buffer is the pointer to buffer address
 *
 * @retval RTEMS_SUCCESSFUL if successful or error code if unsuccessful
 */
rtems_status_code rtems_partition_get_buffer(
  rtems_id   id,
  void     **buffer
);

/**
 *  @brief rtems_partition_return_buffer
 *
 *  This routine implements the rtems_partition_return_buffer directive.  It
 *  frees the buffer to the partition associated with ID.  The buffer must
 *  have been previously allocated from the same partition.
 */
rtems_status_code rtems_partition_return_buffer(
  rtems_id  id,
  void     *buffer
);

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
/* end of include file */
