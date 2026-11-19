#ifndef SIM_FATFS_FF_H_
#define SIM_FATFS_FF_H_

/*
 * Host stand-in for ChaN's FatFs (ff.h), backed by a folder on the laptop.
 *
 * Same names, types and return codes as the real FatFs, so modules/sd_card
 * compiles unchanged against either one: the flight build links the real
 * FatFs + SD SPI diskio, the native build links this. Control the simulated
 * card (folder, faults, power cuts) with sim/sim_sd.h.
 *
 * Only the subset the logger needs is provided. Add functions as needed.
 */

#include <stddef.h>
#include <stdint.h>

typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef unsigned int UINT;
typedef char TCHAR;
typedef DWORD FSIZE_t;

typedef enum {
    FR_OK = 0,
    FR_DISK_ERR,
    FR_INT_ERR,
    FR_NOT_READY,
    FR_NO_FILE,
    FR_NO_PATH,
    FR_INVALID_NAME,
    FR_DENIED,
    FR_EXIST,
    FR_INVALID_OBJECT,
    FR_WRITE_PROTECTED,
    FR_INVALID_DRIVE,
    FR_NOT_ENABLED,
    FR_NO_FILESYSTEM,
    FR_MKFS_ABORTED,
    FR_TIMEOUT,
    FR_LOCKED,
    FR_NOT_ENOUGH_CORE,
    FR_TOO_MANY_OPEN_FILES,
    FR_INVALID_PARAMETER,
} FRESULT;

#define FA_READ          0x01
#define FA_WRITE         0x02
#define FA_OPEN_EXISTING 0x00
#define FA_CREATE_NEW    0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS   0x10
#define FA_OPEN_APPEND   0x30

#define FF_MAX_LFN 255

#define FF_MIN_SS 512

typedef struct {
    BYTE mounted;
    WORD csize;     /* sectors per cluster, as in FatFs */
    DWORD n_fatent; /* clusters + 2, as in FatFs */
} FATFS;

typedef struct {
    int slot;          /* simulator's open-file slot, -1 = closed */
    DWORD generation;  /* stale handle after a power cut / card removal */
    BYTE mode;
    FSIZE_t fptr;
    FSIZE_t size;
} FIL;

typedef struct {
    FSIZE_t fsize;
    TCHAR fname[FF_MAX_LFN + 1];
} FILINFO;

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt);
FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode);
FRESULT f_close(FIL *fp);
FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br);
FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw);
FRESULT f_lseek(FIL *fp, FSIZE_t ofs);
FRESULT f_sync(FIL *fp);
FRESULT f_stat(const TCHAR *path, FILINFO *fno);
FRESULT f_unlink(const TCHAR *path);
FRESULT f_mkdir(const TCHAR *path);
FRESULT f_getfree(const TCHAR *path, DWORD *nclst, FATFS **fatfs);

#define f_size(fp) ((fp)->size)
#define f_tell(fp) ((fp)->fptr)
#define f_eof(fp)  ((int)((fp)->fptr == (fp)->size))

#endif /* SIM_FATFS_FF_H_ */
