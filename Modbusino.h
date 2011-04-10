#ifndef Modbusino_h
#define Modbusino_h

#include <inttypes.h>
#include "WProgram.h"

#define MODBUS_RTU_MAX_ADU_LENGTH  256

class Modbusino {
public:
    Modbusino(void);
    void set_slave(int slave);
    void mapping_new(int nb_register);
    void mapping_free(void);
    uint16_t *get_mapping(void);
    int receive(uint8_t *req);
    int reply(uint8_t *req, int req_length);
private:
    int _slave;
    int _nb_register;
    uint16_t *_tab_register;
    boolean _debug;
    int _receive_msg(uint8_t *msg, int msg_type);
    uint8_t _compute_meta_length_after_function(int function, int msg_type);
    int _compute_data_length_after_meta(uint8_t *msg, int msg_type);
    int _check_integrity(uint8_t *msg, const int msg_length);
    int _build_response_basis(int function, int slave, uint8_t* rsp);
    int _send_msg(uint8_t *req, int req_length);
};

#endif
