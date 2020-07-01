#include <stdio.h>
#include <rc/dsm.h>

// just run the bind function
int main()
{
        // if(rc_dsm_init()==-1){
        //     fprintf(stderr,"failed to start initialize DSM\n");
        //     return -1;
        // }
        // run the built-in bind routine
        rc_dsm_bind_routine();
        return 0;
}
