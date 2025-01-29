#include <string.h>

#include "gateway.h"

/* Build response ack gateway message */
node_gateway_msg_t build_response_ack_gateway_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_gateway_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, GATEWAY, id_node, id_msg, mac, ACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}

/* Build response nack gateway message */
node_gateway_msg_t build_response_nack_gateway_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_gateway_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, GATEWAY, id_node, id_msg, mac, NACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}