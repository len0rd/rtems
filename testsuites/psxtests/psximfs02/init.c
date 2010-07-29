/*
 *  COPYRIGHT (c) 1989-2010.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include <tmacros.h>
#include "test_support.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <rtems/libio.h>

extern Heap_Control  *RTEMS_Malloc_Heap;

rtems_task Init(
  rtems_task_argument argument
)
{
  int status = 0;
  void *alloc_ptr = (void *)0;
  Heap_Information_block Info;
  char linkname_n[20] = {0};
  char linkname_p[20] = {0};
  int i;
  struct stat stat_buf;

  puts( "\n\n*** TEST IMFS 02 ***" );

  puts( "Creating directory /dir00" );
  status = mkdir( "/dir00", S_IRWXU );
  rtems_test_assert( status == 0 );

  puts( "Creating directory /dir00/dir01" );
  status = mkdir( "/dir00/dir01", S_IRWXU );
  rtems_test_assert( status == 0 );

  puts( "Changing directory to /dir00" );
  status = chdir( "/dir00" );
  rtems_test_assert( status == 0 );

  puts( "Creating link dir01-link0 for dir01" );
  status = link( "dir01", "dir01-link0" );
  rtems_test_assert( status == 0 );

  for( i = 1 ; ; ++i ) {
    sprintf( linkname_p, "dir01-link%d", i-1 );
    sprintf( linkname_n, "dir01-link%d", i );
    printf( "\nCreating link %s for %s\n", linkname_n, linkname_p );
    status = link( linkname_p, linkname_n );
    if( status != 0 ) {
      puts("Link creation failed" );
      break;
    }
  }

  puts( "Creating a regular node /node, RDONLY" );
  status = mknod( "/node", S_IFREG | S_IRUSR, 0LL );
  rtems_test_assert( status == 0 );

  puts( "Creating link /node-link for /node" );
  status = link( "/node" , "/node-link" );
  rtems_test_assert( status == 0 );

  puts( "Opening /node-link in WRONLY mode -- expect EACCES" );
  status = open( "/node-link", O_WRONLY );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == EACCES );

  puts( "Creating a symlink /node-slink for /node" );
  status = symlink( "/node" , "/node-slink" );
  rtems_test_assert( status == 0 );

  puts( "Opening /node-slink in WRONLY mode -- expect EACCES" );  
  status = open( "/node-slink", O_WRONLY );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == EACCES );

  puts( "Allocate most of heap" );
  _Heap_Get_information( RTEMS_Malloc_Heap, &Info );
  alloc_ptr = malloc( Info.Free.largest - 150 );

  puts( "Attempt to mount a fs at /dir01 -- expect ENOMEM" );
  status = mount( NULL,
		  "dir01",
		  "imfs",
		  RTEMS_FILESYSTEM_READ_WRITE,
		  NULL );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == ENOMEM );

  puts( "Freeing allocated memory" );
  free( alloc_ptr );

  puts( "Allocate most of heap" );
  _Heap_Get_information( RTEMS_Malloc_Heap, &Info );
  alloc_ptr = malloc( Info.Free.largest - 4 );

  puts( "Changing directory to /" );
  status = chdir( "/" );
  rtems_test_assert( status == 0 );

  puts( "Attempt to create /node-link-2 for /node -- expect ENOMEM" );
  status = link( "/node", "/node-link-2" );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == ENOMEM );

  puts( "Attempt to create /node-slink-2 for /node -- expect ENOMEM" );
  status = symlink( "/node", "node-slink-2" );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == ENOMEM );

  puts( "Freeing allocated memory" );
  free( alloc_ptr );

  puts( "Allocate most of heap" );
  alloc_ptr = malloc( Info.Free.largest - 40 );

  puts( "Attempt to create /node-slink-2 for /node -- expect ENOMEM" );
  status = symlink( "/node", "node-slink-2" );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == ENOMEM );

  puts( "Attempt to stat a hardlink -- expect ENOTSUP" );
  status = lstat( "/node-link", &stat_buf );
  rtems_test_assert( status == -1 );
  rtems_test_assert( errno == ENOTSUP );

  puts( "*** END OF TEST IMFS 02 ***" );
  rtems_test_exit(0);
}

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS                  1
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 4
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
/* end of file */
