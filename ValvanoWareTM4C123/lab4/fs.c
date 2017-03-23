//p-code for fs impl

#include "os.h"

#define BLK_SIZ_BYTES 512
#define BLK_SIZ_WORDS BLK_SIZ_BYTES / sizeof(int)

struct block {
    int dat[BLK_SIZ_WORDS];
};

struct inode {
    int isDir;
    int siz;
    int tstmp;
    struct block *blks;
    char nam[BLK_SIZ_BYTES - 4 * sizeof(int)];
};

struct mem_inode {
    struct inode *inod;
    int fd;
    int opnrs;
    int rdrs;    
    Sema4Type const rlok;
    Sema4Type const wlok;
};

#define MAX_OPND 10
static struct mem_inode inods[MAX_OPND];
static int opnd_ctr = 0;

static int fd_ctr = 0;

static Sema4Type fs_lok;

int fs_init(void) {
    OS_InitSemaphore(&fs_lok, 1);
    for(int i = 0; i < MAX_OPND; ++i)
        inods[i].fd = -1;
}

int fs_fopn(const char *nam) {
    int ret;
    
    OS_bWait(&fs_lok);
    if(opnd_ctr == MAX_OPND) {
        ret = -1;
        goto finally;
    } else {
        for(int i = 0; i < MAX_OPND; ++i)
            if(strcmp(inods[i], nam) == 0) {
                ++inods[i].opnrs;
                ret = inods[i].fd;
                goto finally;
            }        
        struct mem_inode cur = inods[opnd_ctr++];
        
        cur.inod = disk_open(nam);
        ret = cur.fd = fd_ctr++;
        cur.opnrs = 1;
        cur.rdrs = 0;
        OS_SemaphoreInit(&cur.rlok, 1);
        OS_SemaphoreInit(&cur.wlok, 1);
    }    
    finally:
        OS_bSignal(&fs_lok);
        return ret;
}

int fs_fclos(int fd) {
    int ret;
    
    OS_bWait(&fs_lok);
    if(inods[fd].opnrs == 0) {
        ret = -1;
        goto finally;
    }
    
    struct mem_inode cur = inods[fd];

    OS_bWait(&cur.rlok);
    OS_bWait(&cur.wlok);
    if(inods[fd].opnrs > 0)
        --inods[fd].opnrs;
    else
        --opnd_ctr;
    OS_bWait(&cur.rlok);
    OS_bWait(&cur.wlok);
        
    finally:
        OS_bSignal(&fs_lok);
        return ret;
}

int fs_fread(int fd, char *buf, int off, int siz) {
    struct mem_inode cur = inods[fd];
    
    OS_bWait(&cur.rlok);
    if(++cur.rdrs == 1)
        OS_bWait(&cur.wlok);    
    OS_bSignal(&cur.rlok);
    
    //read logic
    int dat;
    
    OS_bWait(&cur.rlok);
    if(--cur.rdrs == 0)
        OS_bSignal(&cur.wlok);
    OS_bSignal(&cur.rlok);
    
    return dat;
}

int fs_fwrite(int fd, char *buf, int off, int size) {
    struct mem_inode cur = inods[fd];
    
    OS_bWait(&cur.wlok);
    
    //write logic
    
    OS_bSignal(&cur.wlok);
}
