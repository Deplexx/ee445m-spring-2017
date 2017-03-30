// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Jonathan W. Valvano 3/9/17

#include <string.h>
#include "edisk.h"
#include "efile.h"
#include "../lab3/UART.h"
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
static struct block fils;

struct mem_inode {
    struct inode inod;
    int fd;
    int opnrs;
    int rdrs;
    Sema4Type rlok;
    Sema4Type wlok;
};


#define MAX_OPND
static int byte_ctr;
static int opnd_fil;
static struct inode cur_inod;
static struct block cur_blks;
static struct block cur_dat;

static Sema4Type fs_lok;


static void clr_blk(struct block *blk, int num);
static int find_file(const char* nam);
static int next_free_blk(void);

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
    eDisk_Init(0);

    OS_InitSemaphore(&fs_lok, 0);

    eDisk_ReadBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    eDisk_ReadBlock((BYTE*) &fils, root.blks);
    return SUCCESS;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
    disk_ioctl(0, GET_SECTOR_COUNT, &supr_blk.fs_sctrs);
    disk_ioctl(0, GET_BLOCK_SIZE, &supr_blk.fs_blk_siz);
    clr_blk(&blk_bmap, 0);
    blk_bmap.dat[0] |= 0x07; //first 3 blocks reserved for metadata
    clr_blk((struct block*) &root, 0);
    root.isDir = 1;
    strcpy(root.nam, "/");
    root.siz = 0;
    int blks = root.blks = next_free_blk();
    clr_blk(&fils, -1);

    eDisk_WriteBlock((BYTE*) &supr_blk, SUPR_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &blk_bmap, BLK_BMAP_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &root, ROOT_INOD_BLK_NUM);
    eDisk_WriteBlock((BYTE*) &fils, blks);

    return SUCCESS;   // OK
}



//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty
    OS_bWait(&fs_lok);
    static struct inode inod;
    static struct block blks;
    clr_blk(&blks, -1);
    int inod_blks = inod.blks = next_free_blk();
    inod.siz = 0;
    strcpy(inod.nam, name);
    int fil_blk;
    eDisk_WriteBlock((BYTE*) &inod, fil_blk = next_free_blk());
    eDisk_WriteBlock((BYTE*) &blks, inod_blks);

    fils.dat[root.siz] = fil_blk;
    ++root.siz;

    eFile_Close();
    OS_bSignal(&fs_lok);

    return SUCCESS;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]){      // open a file for writing
    OS_bWait(&fs_lok);
    int inod_blk = find_file(name);
    opnd_fil = inod_blk;
    eDisk_ReadBlock((BYTE*) &cur_inod, inod_blk);
    eDisk_ReadBlock((BYTE*) &cur_blks, cur_inod.blks);
    eDisk_ReadBlock((BYTE*) &cur_dat, cur_blks.dat[cur_inod.siz / BLK_SIZ_BYTES]);
    byte_ctr = cur_inod.siz;

    OS_bSignal(&fs_lok);
    return 0;
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(char data){
    //printf("write");
    OS_bWait(&fs_lok);
    //UART_OutChar(data);
    if((cur_inod.siz % BLK_SIZ_BYTES) == 0 ) {
        int new_blk_num = next_free_blk();
        cur_blks.dat[cur_inod.siz / BLK_SIZ_BYTES] = new_blk_num;
        eDisk_WriteBlock((BYTE*) &cur_blks, cur_inod.blks);
        clr_blk(&cur_dat, 0);
    }
    ((char*) cur_dat.dat)[cur_inod.siz % BLK_SIZ_BYTES] = data;
    eDisk_WriteBlock((BYTE*) &cur_dat, cur_blks.dat[cur_inod.siz++ / BLK_SIZ_BYTES]);
    eDisk_WriteBlock((BYTE*) &cur_inod, opnd_fil);

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
    eDisk_WriteBlock((BYTE*) &fils, root.blks);
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
    OS_bWait(&fs_lok);
    byte_ctr = 0;
    int inod_blk = find_file(name);
    opnd_fil = inod_blk;
    eDisk_ReadBlock((BYTE*) &cur_inod, inod_blk);
    eDisk_ReadBlock((BYTE*) &cur_blks, cur_inod.blks);
    eDisk_ReadBlock((BYTE*) &cur_dat, cur_blks.dat[0]);

    OS_bSignal(&fs_lok);
    return 0;
}
 
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt) {       // get next byte
    OS_bWait(&fs_lok);
//    UART_OutString("bytectr: "); UART_OutUDec(byte_ctr); UART_OutCRLF();
//    UART_OutString("cur_inod.siz: "); UART_OutUDec(cur_inod.siz); UART_OutCRLF();
    if(byte_ctr >= cur_inod.siz) {
        byte_ctr = 0;
        if(byte_ctr > BLK_SIZ_BYTES)
            eDisk_ReadBlock((BYTE*) &cur_dat, cur_blks.dat[0]);
        return 1;
    }
    *pt = ((char*) cur_dat.dat)[byte_ctr++ % BLK_SIZ_BYTES];
//    UART_OutChar(*pt);
    if(((byte_ctr % BLK_SIZ_BYTES) == 0 )&& (byte_ctr != 0))
        eDisk_ReadBlock((BYTE*) &cur_dat, cur_blks.dat[byte_ctr / BLK_SIZ_BYTES]);
    OS_bSignal(&fs_lok);
    return 0;
}

    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
    byte_ctr = 0;
    return SUCCESS;
}




//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char*)){
    OS_bWait(&fs_lok);
    for(int i = 0; i < root.siz; ++i) {
        static struct inode fil;
        eDisk_ReadBlock((BYTE*) &fil, fils.dat[i]);
        fp(fil.nam);
        fp("\n");
        fp("\r");
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
    int fil_blk = find_file(name);
    if(fil_blk == -1) {
        OS_bSignal(&fs_lok);
        return -1;
    }
    for(int i = 0; i < root.siz; ++i)
        if(fil_blk == fils.dat[i]) {
            for(int j = i + 1; j < root.siz; ++j)
                fils.dat[j - 1] = fils.dat[j];
            --root.siz;
            OS_bSignal(&fs_lok);
            return SUCCESS;
        }
    OS_bSignal(&fs_lok);
    return FAIL;    // restore directory back to flash
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

static void clr_blk(struct block *blk, int num) {
    for(int i = 0; i < BLK_SIZ_WORDS; ++i)
        blk->dat[i] = num;
}

//assumption: filesystem is locked when calling this function
static int find_file(const char* nam) {
    static struct inode tmp;
    for(int i = 0; i < root.siz; ++i) {
        eDisk_ReadBlock((BYTE*) &tmp, fils.dat[i]);
        if(strcmp(tmp.nam, nam) == 0)
            return fils.dat[i];
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
