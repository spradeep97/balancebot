#define _GNU_SOURCE
#include <lcm/lcm.h>
#include "l2g_t.h"
#include <time.h>

void on_l2g(const lcm_recv_buf_t *rbuf, const char *channel,
            const l2g_t *msg, void *userdata) {
    printf("%.2f %.2f %.2f\n", msg->l2g[0], msg->l2g[1], msg->l2g[2]);
}

double seconds_now(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) {
        fprintf(stderr, "Retrieving system time failed.\n");
        exit(1);
    }
    return now.tv_sec + now.tv_nsec / 1000000000.0;
}

int main(void) {
    lcm_t *lcm = lcm_create(NULL);

    l2g_t_subscription_t *l2g_sub = l2g_t_subscribe(lcm, "L2G", on_l2g, NULL);
    l2g_t message = {0};
    message.utime = 0;
    message.l2g[0] = 1.0;
    message.l2g[1] = 2.0;
    message.l2g[2] = 3.0;
    l2g_t_publish(lcm, "L2G", &message);
    double start = seconds_now();
    while (seconds_now() - start < 0.5) {
        lcm_handle_timeout(lcm, 100);
    }
    l2g_t_unsubscribe(lcm, l2g_sub);
    
    lcm_destroy(lcm);
    return 0;
}
