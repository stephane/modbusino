#ifndef Modbusino_h
#define Modbusino_h

#include <inttypes.h>
#include "WProgram.h"

#define MODBUS_RTU_MAX_ADU_LENGTH  256

/* Protocol exceptions */
enum {
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
    MODBUS_EXCEPTION_ACKNOWLEDGE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
    MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
    MODBUS_EXCEPTION_MEMORY_PARITY,
    MODBUS_EXCEPTION_NOT_DEFINED,
    MODBUS_EXCEPTION_GATEWAY_PATH,
    MODBUS_EXCEPTION_GATEWAY_TARGET,
    MODBUS_EXCEPTION_MAX
};

class Modbusino {
public:
    Modbusino(void);
    void set_slave(int slave);
    void mapping_new(int nb_register);
    void mapping_free(void);
    uint16_t *get_mapping(void);
    int receive(uint8_t *req);
    int reply(uint8_t *req, int req_length);
    uint16_t *tab_reg;
private:
    int _slave;
    int _nb_register;
    boolean _debug;
    int _receive_msg(uint8_t *msg, int msg_type);
    uint8_t _compute_meta_length_after_function(int function, int msg_type);
    int _compute_data_length_after_meta(uint8_t *msg, int msg_type);
    int _check_integrity(uint8_t *msg, const int msg_length);
    int _build_response_basis(int function, int slave, uint8_t* rsp);
    int _send_msg(uint8_t *req, int req_length);
    int _response_exception(int slave, int function, int exception_code,
			   uint8_t *rsp);
};

#endif
