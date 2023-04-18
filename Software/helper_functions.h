void uint32_to_str(char * str, uint32_t num);
uint8_t ReadCalibrationByte( uint8_t index );
void int_to_5digit(int num, char * buffer);
void int_to_4digit(int num, char * buffer);
void int_to_3digit(int num, char * buffer);
void int_to_4digit_nodec(int num, char * buffer);
void int_to_hex(char * str, int num);
void calc_crc8(can_frame_t *frame);
void calc_sum4(can_frame_t *frame);
