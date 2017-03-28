// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Jonathan W. Valvano 3/9/17

#include <string.h>
#include "edisk.h"
#include "efile.h"
#include "UART2.h"
#include <stdio.h>
#include <stdlib.h>

#include "os.h"

#define SUCCESS 0
#define FAIL 1

#define BLKS 8388608 // 4GB / BLK_SIZ_BYTES
#define BLK_SIZ_BYTES 512
#define BLK_SIZ_WORDS BLK_SIZ_BYTES / sizeof(int)

struct block {
    int dat[BLK_SIZ_WORDS];
};

#define SUPR_BLK_NUM 0
#define ROOT_INOD_BLK_NUM 1
#define BLK_BMAP_BLK_NUM 2

struct super {
    int fs_sctrs;
    int fs_blk_siz;
    char unused[BLK_SIZ_BYTES - 2 * sizeof(int)];
};

struct super supr_blk;
struct block blk_bmap;

#define INOD_BLKS BLK_SIZ_BYTES - 4 * sizeof(int)
struct inode {
    int isDir;
    int siz;
    int tstmp;
    int blks;
    char nam[INOD_BLKS];
};

struct mem_inode {
    struct inode inod;
    int fd;
    int opnrs;
    int rdrs;
    Sema4Type rlok;
    Sema4Type wlok;
};


#define MAX_OPND 10
static struct mem_inode inods[MAX_OPND];
static int opnd_ctr = 0;

static int fd_ctr = 0;

static Sema4Type fs_lok;


static struct inode pwd;


static void clr_blk(struct block *blk);
static void find_file(const char* nam, struct inode *inod);
static int next_free_blk(void);
static int byte2blk(struct mem_inode *inod, int off);
static int byte2off(struct mem_inode *inod, int off);
static struct mem_inode *nxt_avail_inod(void);

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
    CS_Init();
    eDisk_Init(0);

    OS_InitSemaphore(&fs_lok, 1);
    for(int i = 0; i < MAX_OPND; ++i)
        inods[i].fd = -1;

    eDisk_ReadBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);

    return SUCCESS;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
    disk_ioctl(0, GET_SECTOR_COUNT, &supr_blk.fs_sctrs);
    disk_ioctl(0, GET_BLOCK_SIZE, &supr_blk.fs_blk_siz);
    eDisk_WriteBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);

    clr_blk(&blk_bmap);
    eDisk_WriteBlock((BYTE*) &blk_bmap, SUPR_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);

    return SUCCESS;   // OK
}



//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty
    struct inode inod;
    inod.blks = NULL;
    inod.siz = 0;
    strcpy(inod.nam, name);
    return eDisk_WriteBlock((BYTE*) &inod, next_free_blk());
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]){      // open a file for writing
  return eFile_ROpen(name);
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(char data){
    struct block blk;

    struct mem_inode *fil = &inods[OS_GetOpenedFile()];
    eDisk_ReadBlock((BYTE*) &blk, byte2blk(fil, fil->inod.siz + 1));
    ((char *) &blk.dat)[byte2off(fil, fil->inod.siz + 1)] = data;
    eDisk_WriteBlock((BYTE*) &blk, byte2blk(fil, fil->inod.siz + 1));
    ++fil->inod.siz;
    return SUCCESS;
}


//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){
    return SUCCESS;
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
  return eFile_RClose();
}


//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( char name[]){      // open a file for reading 
    int ret;

    OS_bWait(&fs_lok);
    if(opnd_ctr == MAX_OPND) {
        ret = 1;
        goto finally;
    } else {
        for(int i = 0; i < MAX_OPND; ++i)
            if(strcmp(inods[i].inod.nam, name) == 0) {
                ++inods[i].opnrs;
                ret = 0;
                OS_SetOpenedFile(inods[i].fd);
                goto finally;
            }
        ++opnd_ctr;
        struct mem_inode *cur = nxt_avail_inod();
        find_file(name, &cur->inod);
        cur->fd = fd_ctr++;
        OS_SetOpenedFile(cur->fd);
        OS_SetCurByte(0);
        cur->opnrs = 1;
        cur->rdrs = 0;
        OS_InitSemaphore(&cur->rlok, 0);
        OS_InitSemaphore(&cur->wlok, 0);
        ret = 0;
    }
    finally:
        OS_bSignal(&fs_lok);
        return ret;
}
 
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
    struct block blk;
    OS_SetCurByte(OS_GetCurByte() + 1);
    eDisk_ReadBlock((BYTE*) &blk, byte2blk(&inods[OS_GetOpenedFile()], OS_GetCurByte()));
    *pt = ((char*) blk.dat)[byte2off(&inods[OS_GetOpenedFile()], OS_GetCurByte())];
    return SUCCESS;
}

    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
    OS_SetOpenedFile(inods[OS_GetOpenedFile()].fd = -1);
    return SUCCESS;
}




//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char)){   
    struct inode root;
    struct block root_fils;
    eDisk_ReadBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &root_fils, root.blks);
    for(int i = 0; i < INOD_BLKS; ++i) {
        struct inode fil;
        eDisk_ReadBlock((BYTE*) &fil, root_fils.dat[i]);
        printf(fil.nam);
        printf("\n");
    }
    return SUCCESS;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]){  // remove this file 

  return SUCCESS;    // restore directory back to flash
}

int StreamToFile=0;                // 0=UART, 1=stream to file

int eFile_RedirectToFile(char *name){
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToFile = 1;
  return 0;
}

int eFile_EndRedirectToFile(void){
  StreamToFile = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int fputc (int ch, FILE *f) { 
  if(StreamToFile){
    if(eFile_Write(ch)){          // close file on error
       eFile_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }

   // regular UART output
  UART_OutChar(ch);
  return 0; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);            // echo
  return ch;
}

static void clr_blk(struct block *blk) {
    for(int i = 0; i < BLK_SIZ_WORDS; ++i)
        blk->dat[i] = 0;
}

//assumption: filesystem is locked when calling this function
static void find_file(const char* nam, struct inode *inod) {
    struct block fils;
    struct inode fil;
    eDisk_ReadBlock((BYTE*) &pwd, ROOT_INOD_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &fils, pwd.blks);
    for(int i = 0; i < BLK_SIZ_WORDS; ++i) {
        eDisk_ReadBlock((BYTE*) &fil, fils.dat[i]);
        if(strcmp(fil.nam, nam) == 0)
            memcpy(inod, &fil, BLK_SIZ_BYTES);
    }
}

static int next_free_blk(void) {
    int c = 0;
    for(int i = 0; i < BLK_SIZ_BYTES; ++i)
        for(int j = 0; j < 8; ++j)
            if((blk_bmap.dat[i] & (1 << j)) == 0) {
                blk_bmap.dat[i] |= 1 << j;
                return c;
            } else ++c;
    return -1;
}

static int byte2blk(struct mem_inode *inod, int off) {
    struct block blks;
    eDisk_ReadBlock((BYTE*) &blks, inod->inod.blks);
    return blks.dat[off / BLK_SIZ_BYTES];
}
static int byte2off(struct mem_inode *inod, int off) {
    return off % BLK_SIZ_BYTES;
}

static struct mem_inode *nxt_avail_inod(void) {
    for(int i = 0; i < MAX_OPND; ++i)
        if(inods[i].fd == -1)
            return &inods[i];
    return NULL;
}
