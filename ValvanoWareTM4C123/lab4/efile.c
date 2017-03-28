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

static struct super supr_blk;
static struct block blk_bmap;

#define INOD_BLKS BLK_SIZ_BYTES - 4 * sizeof(int)
struct inode {
    int isDir;
    int siz;
    int tstmp;
    int blks;
    char nam[INOD_BLKS];
};

static struct inode root;

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


static void clr_blk(struct block *blk);
static int find_file(const char* nam, struct inode *inod);
static int next_free_blk(void);
static int byte2off(int off);
static struct mem_inode *nxt_avail_inod(void);
static int get_blk(struct inode *fil, struct block *buf, int off);

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
    eDisk_Init(0);

    OS_InitSemaphore(&fs_lok, 0);
    for(int i = 0; i < MAX_OPND; ++i)
        inods[i].fd = -1;

    eDisk_ReadBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    return SUCCESS;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
    disk_ioctl(0, GET_SECTOR_COUNT, &supr_blk.fs_sctrs);
    disk_ioctl(0, GET_BLOCK_SIZE, &supr_blk.fs_blk_siz);
    clr_blk(&blk_bmap);
    blk_bmap.dat[0] |= 0x07; //first 3 blocks reserved for metadata
    clr_blk((struct block*) &root);
    root.isDir = 1;
    strcpy(root.nam, "/");
    root.siz = 0;
    root.blks = next_free_blk();

    eDisk_WriteBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);

    return SUCCESS;   // OK
}



//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty
    OS_bWait(&fs_lok);
    static struct inode inod;
    static struct block root_blks;
    inod.blks = -1;
    inod.siz = 0;
    strcpy(inod.nam, name);
    int fil_blk;
    eDisk_WriteBlock((BYTE*) &inod, fil_blk = next_free_blk());

    int blk_num = get_blk(&root, &root_blks, root.siz += sizeof(int));
    root_blks.dat[(root.siz - sizeof(int)) / sizeof(int)] = fil_blk;
    eDisk_WriteBlock((BYTE*) &root_blks, root.blks);
    eDisk_WriteBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    OS_bSignal(&fs_lok);

    return SUCCESS;
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
    static struct block blk;
    OS_bWait(&fs_lok);
    struct mem_inode *fil = &inods[OS_GetOpenedFile()];
    int blk_num = get_blk(&fil->inod, &blk, fil->inod.siz + 1);
    ((char *) &blk.dat)[byte2off(fil->inod.siz + 1)] = data;
    eDisk_WriteBlock((BYTE*) &blk, blk_num);
    OS_bSignal(&fs_lok);
    return SUCCESS;
}


//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){
    eDisk_WriteBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
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
    static struct block blk;
    OS_bWait(&fs_lok);
    OS_SetCurByte(OS_GetCurByte() + 1);
    get_blk(&inods[OS_GetOpenedFile()].inod, &blk, OS_GetCurByte());
    *pt = ((char*) blk.dat)[byte2off(OS_GetCurByte())];
    OS_bSignal(&fs_lok);
    return SUCCESS;
}

    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
    OS_bWait(&fs_lok);
    OS_SetOpenedFile(inods[OS_GetOpenedFile()].fd = -1);
    OS_bSignal(&fs_lok);
    return SUCCESS;
}




//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char*)){
    OS_bWait(&fs_lok);
    static struct block root_fils;
    eDisk_ReadBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    if(root.siz > 0) {
    eDisk_ReadBlock((BYTE*) &root_fils, root.blks);
        for(int i = 0; i < INOD_BLKS; ++i) {
            static struct inode fil;
            eDisk_ReadBlock((BYTE*) &fil, root_fils.dat[i]);
            fp(fil.nam);
            fp("\n");
            fp("\r");
        }
    }
    OS_bSignal(&fs_lok);
    return SUCCESS;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]){  // remove this file 
    OS_bWait(&fs_lok);
    int ret;
    static struct block root_fils, blk;
    static struct inode fil;
    int blk_num = find_file(name, &fil);
    if(blk_num == -1) {
        ret = 0;
        goto finally;
    }
    eDisk_ReadBlock((BYTE*) &root_fils, root.blks);
    int j = 0;
    for(int i = root_fils.dat[0]; i != -1; i = root_fils.dat[++j]) {
        eDisk_ReadBlock((BYTE*) &blk, i);
        for(int k = 0; k < BLK_SIZ_WORDS; ++k) {
            if(blk.dat[k] == blk_num) {
                blk.dat[k] = -1;
                eDisk_WriteBlock((BYTE*) &blk, i);
                ret = 0;
                goto save;
            }
        }
    }

    save:
        if(ret == 0) {
            eDisk_WriteBlock((BYTE*) &root_fils, root.blks);
            eDisk_WriteBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
        }
    finally:
        OS_bSignal(&fs_lok);
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
static int find_file(const char* nam, struct inode *inod) {
    static struct block fils, blk;
    static struct inode fil;
    eDisk_ReadBlock((BYTE*) &fils, root.blks);
    for(int i = 0; i < BLK_SIZ_WORDS * BLK_SIZ_WORDS; ++i) {
        int ret = get_blk(&root, &blk, i / 4);
        eDisk_ReadBlock((BYTE*) &fil, blk.dat[byte2off(i) / 4]);
        if(strcmp(fil.nam, nam) == 0) {
            memcpy(inod, &fil, BLK_SIZ_BYTES);
            return ret;
        }
    }
    return -1;
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

static int byte2off(int off) {
    return off % BLK_SIZ_BYTES;
}

static struct mem_inode *nxt_avail_inod(void) {
    for(int i = 0; i < MAX_OPND; ++i)
        if(inods[i].fd == -1)
            return &inods[i];
    return NULL;
}

static int get_blk(struct inode *fil, struct block *buf, int off) {
    int blk_i = fil->siz / BLK_SIZ_BYTES;

    if(fil->blks == -1) {
        fil->blks = next_free_blk();
        for(int i = 0; i < BLK_SIZ_WORDS; ++i)
            buf->dat[i] = -1;
    } else eDisk_ReadBlock((BYTE*) buf, fil->blks);

    for(int i = 0; i <= blk_i; ++i) {
        int new_blk = 0;
        if(buf->dat[blk_i] == -1) {
            new_blk = 1;
            buf->dat[blk_i] = next_free_blk();
            eDisk_WriteBlock((BYTE*) buf, fil->blks);
        }
        if(i == blk_i) {
            int ret = buf->dat[blk_i];
            if(!new_blk)
                eDisk_ReadBlock((BYTE*) buf, ret);
            else clr_blk(buf);
            return ret;
        }
    }

    return -1;
}
