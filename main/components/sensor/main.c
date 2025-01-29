#include <stdio.h>
#include <string.h>
#include "sensors.h"

int main() {

    uint8_t a[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    node_id_alarm n, n1, n2, n3;
    n.header.id = 10;
    n.header.cmd = GET;
    n.header.node = ALARM_SENSOR;

    n.battery_low_detect = false;
    n.crc = 1232;
    n.state = false;
    n.time = 1234;
    
    memcpy(n.header.mac, a, sizeof(6));

    n1 = n;
    n1.header.id = 11;

    n2 = n;
    n2.header.id = 12;

    n3 = n;
    n3.header.id = 13;

    printf("Start test\n");

    struct node_id_alarm_list *p = NULL;
    p = add_node_id(p, n);
    p = add_node_id(p, n1);
    p = add_node_id(p, n2);
    p = del_node_id(p, 12);
    p = add_node_id(p, n3);
    p = del_node_id(p, 13);
    p = add_node_id(p, n3);
    printf("Counter: %d\n", get_num_node_id_list());

    print_list(p);

    return 0;
}