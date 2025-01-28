#include <string.h>

#include "header.h"

/* Build header message */
node_id_header_t build_header_msg(enum msg_type msg, enum node_type node, uint8_t id_node, uint8_t id_msg, uint8_t mac[], enum cmd_type cmd) {

    node_id_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    hdr.msg = msg;
    hdr.id_node = id_node;
    hdr.id_msg = id_msg;
    memcpy(hdr.mac, mac, MAC_SIZE);
    hdr.cmd = cmd;

    return hdr;
}