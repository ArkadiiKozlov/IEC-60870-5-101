#ifndef _MEK_101_DEVICE_H_
#define _MEK_101_DEVICE_H_

#include "messages.h"
#include "station.h"
#include "../uart_simple.h"

#include <map>
#include <vector>
#include <string>

void print_time ();

using namespace std;
#define START_WORD        0x68
#define END_WORD          0x16
enum PROT_STATE { START = 0, ESTABLISHED };
//////////////////////////////////////////////////////////////////////////////////////
struct FT1_2_HEAD {

             FT1_2_HEAD () {
                      st_word = 0x68;
                      L = 0x00;               
                      L_R = 0x00;
		      C = 0x00;
                      A = 0x00;
                      st_word_r = 0x68;		      
                   };

  unsigned char st_word; // starting byte
  unsigned char L;       // user data length plus C and A
  unsigned char L_R;     // user data length plus C and A
  unsigned char st_word_r; // repeat starting byte
  unsigned char C;       // control field
  unsigned char A;       // address field

} __attribute((packed));
/////////////////////////////////////////////////////////////////////////////////////
            struct mek_101_ti_buffer {
         	    unsigned short start_address;            
         	    unsigned short end_address;
	            std::vector <float> ti_buffer;         	             	    
	            std::vector <float> ti_buffer_prev;         	             	    	            
	            int ti_st_number;
                    int position_ti;

            };

            struct mek_101_ts_buffer {
         	    unsigned short start_address;
         	    unsigned short end_address;         	    
	            std::vector <unsigned char> ts_buffer;         	             	    
	            std::vector <unsigned char> ts_buffer_prev;         	             	    	            
	            int ts_st_number;
                    int position_ts;	            	            
            };
            

class mek_101_device { 

    private:
            speed_t speed;
	    char tty_name [30];
	    char sport_param [30];
            uart_simple *u_s;
	    MsgN * msg_obj;
            unsigned char my_addr485;
	    unsigned char other_addr485;
	    unsigned char COA;
	    unsigned short ts_group_shifter;
	    unsigned short ti_group_shifter;	    
            unsigned char buff_out[1024];
            unsigned char buff_in[1024];            
            char prot_str[1024];            
            FT1_2_HEAD    h_ft1_2;
            int size_in;                
            int size_out;
            char err_str[100];
            int timeout_first_byte;
            int timeout_next_byte;
            int protocol_state;
            unsigned char last_FCB;
            int thread_num;      
            unsigned short last_tu_address;
            unsigned short last_tu_command;            
            unsigned char r_was_class_a;
            struct tu_features {
                  tu_features () {
                    address = 0x00;
                    command = 0x00;                                    
                    command_exists = 0x00;
                  };
                  
                  unsigned short address;
                  unsigned short command;                                    
                  unsigned short command_exists;
            };
            vector <tu_features> tu_queue;
            int block_TU_period;
            int time_to_sync;
            struct timeval time_begin_block_TU;
            struct timeval time_start_sync;            

    protected:	    
            unsigned char checksum_256 (unsigned char *_buf, unsigned char _size);            
	    STATUS        Check_FT1_2_variable (unsigned char *_buf, int _size);
            STATUS        Check_FT1_2_fixed (unsigned char *_buf, int _size);
            void log_bytes (unsigned char *buffer, int _size, int _in_out);
            void print_bytes (unsigned char *buffer, int _size);
            unsigned char do_control_byte_function (unsigned char _c_b);
            unsigned char do_control_byte_FCB (unsigned char _c_b);            
            STATUS send_time ();
            void compute_reply (vector <mek_101_ts_buffer> &_ts_data, vector <mek_101_ti_buffer> &_ti_data);
            unsigned int get_TS_i_channel (unsigned short _addr, vector <mek_101_ts_buffer> &_ts_data);
            unsigned int get_TI_i_channel (unsigned short _addr, vector <mek_101_ti_buffer> &_ti_data);
            
    public:
            mek_101_device ();
	    virtual ~mek_101_device ();
	    virtual void Set_Msg_Obj (MsgN *_obj, int _thread_num) { msg_obj = _obj; thread_num = _thread_num; };            
	    void Set_u_s_obj (uart_simple *_u_s_obj) { u_s = _u_s_obj; };      
	    STATUS isActive (void);
	    void set_my_addr (unsigned char _my_addr485) { my_addr485 = _my_addr485; };
	    void set_other_addr (unsigned char _other_addr485) { other_addr485 = _other_addr485; };
	    void set_COA (unsigned char _COA) { COA = _COA; };	    
            void set_timeouts (int _timeout_first_byte, int _timeout_next_byte) {
                                timeout_first_byte = _timeout_first_byte;
                                timeout_next_byte  =  _timeout_next_byte;
                              }
            STATUS get_data (vector <mek_101_ts_buffer> &_ts_data, vector <mek_101_ti_buffer> &_ti_data);
            STATUS get_data2 (vector <unsigned char> &_ts_data);
            STATUS set_connection ();
            STATUS perform_tranzaction (unsigned char *_buff_out, int _size);
            void get_garbage ();
            void print_mek101_info (const unsigned char * _buff);
            STATUS send_tu();
            unsigned char get_last_class () { return r_was_class_a; };
            STATUS put_tu_in_queue (unsigned short, unsigned short);
            void reset_tu_queue();
            
};	
#endif /* !__MEK_101_DEVICE_H_ */
