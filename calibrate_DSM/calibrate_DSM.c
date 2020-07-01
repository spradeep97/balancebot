#include <stdio.h>
#include <rc/dsm.h>
int main()
{
        printf("Please connect a DSM satellite receiver and make sure\n");
        printf("your transmitter is on and paired to the receiver.\n");
        printf("\n");
        printf("Press ENTER to continue or anything else to quit\n");
        getchar();
        // run the calibration routine
        rc_dsm_calibrate_routine();
        return 0;
}
