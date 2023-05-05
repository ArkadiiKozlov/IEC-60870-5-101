#include "./mek_101_device.h"
void print_time () {
   tm time_local1; 
   struct timeval now; 
   gettimeofday (&now, NULL); 
   memcpy (&time_local1, localtime (&now.tv_sec), sizeof (tm));	
   printf ("%02d.%03ld:", time_local1.tm_sec, now.tv_usec/1000); 
//   printf ("%02d.%03ld:", time_local1.tm_sec, now.tv_usec); 
}   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define		printf_log(args...)	{ printf (args); printf ("\n"); sprintf (prot_str, args); msg_obj->log (prot_str); }
#define		printf_log2(args...)	{ printf (args); sprintf (prot_str, args); msg_obj->log (prot_str); }

unsigned char request_polling[15]  = { 0x68, 0x09, 0x09, 0x68, 0x73, 0x01, 0x64, 0x01, 0x06, 0x01, 0x00, 0x00, 0x14, 0x00, 0x16 };
unsigned char request_class_1_2[5]  =   { 0x10, 0x5b, 0x00, 0x00, 0x16 };        
unsigned char request_TU[15]  =      { 0x68, 0x09, 0x09, 0x68, 0x73, 0x01,   45, 0x01, 0x06, 0x01, 0x94, 0x11, 0x81, 0x00, 0x16 };

mek_101_device::mek_101_device ()
{ 
  my_addr485 = 0x00;
  other_addr485 = 0x00;
  COA = 0x00;
  timeout_first_byte = 50000;
  timeout_next_byte  = 50000;
  protocol_state = START;
  last_FCB = 0x00;  
  thread_num = 0;
  last_tu_address = 0;
  last_tu_command = 0;
  tu_queue.resize (20);
  block_TU_period = 0;
  time_to_sync = 0;
  gettimeofday (&time_begin_block_TU, NULL);
  time_start_sync = time_begin_block_TU;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::get_data (vector <mek_101_ts_buffer> &_ts_data, vector <mek_101_ti_buffer> &_ti_data)
{
  if (block_TU_period) {	   	        
       struct timeval now; 
       gettimeofday (&now, NULL); 
       if (abs(now.tv_sec - time_begin_block_TU.tv_sec) >= 12) {
            block_TU_period = 0;                        
            last_tu_address = 0;
            last_tu_command = 0;
          }            
       printf_log ("TU is Blocked to %d sec.", 12 - abs(now.tv_sec - time_begin_block_TU.tv_sec));
     }
     
  if (set_connection () == ERROR)
       return ERROR;

  struct timeval now; 
  gettimeofday (&now, NULL); 
  if (abs(now.tv_sec - time_start_sync.tv_sec) >= 3600) {  //sync in 1 hour
       gettimeofday (&time_start_sync, NULL);
       send_time ();
     }
    
  printf_log ("request data class %c", ((request_class_1_2[1]&0x0f) == 0x0a) ? '1' : '2' );
  if (perform_tranzaction (request_class_1_2, 5) == ERROR) {
        protocol_state = START;
        return ERROR;
     }
//  print_time (); printf ("1.1\n");
  
  request_class_1_2[1] = do_control_byte_function (request_class_1_2[1]);
  
  compute_reply (_ts_data, _ti_data);
  
  r_was_class_a = (request_class_1_2[1]&0x0f);  // remember it - for upper layer to call get_data() again without delay if 1 class data available (0x0a)

  send_tu ();
  
  bzero (buff_in, 1024); //should be last
  

  return OK;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mek_101_device::reset_tu_queue()
{

   printf_log ("reseting tu queue");
   for (unsigned int i = 0; i < tu_queue.size (); i++) {

         tu_queue[i].address = 0;
         tu_queue[i].command = 0;                        
         tu_queue[i].command_exists = 0;
       }         

   last_tu_address = 0;
   last_tu_command = 0;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::send_tu()
{

  if (block_TU_period != 0) {
//        printf_log ("TU blocked while..., remain - %d cycles", block_TU_period);
       return ERROR;
     }
    
  for (unsigned int i = 0; i < tu_queue.size (); i++)
       if (tu_queue[i].command_exists == 1) {
            block_TU_period = 1; 
            gettimeofday (&time_begin_block_TU, NULL);
//            printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            printf_log ("sening TU preparing queue: %d address: %04x command: %02x", i, tu_queue[i].address, tu_queue[i].command)
            memcpy (&last_tu_address, &tu_queue[i].address, 2);
            memcpy (&last_tu_command, &tu_queue[i].command, 2);
            memcpy (request_TU + 10, &tu_queue[i].address, 2);
            request_TU[12] = 0x80 | tu_queue[i].command;   
            STATUS err = perform_tranzaction (request_TU, 15); 
            tu_queue[i].address = 0;
            tu_queue[i].command = 0;                        
            tu_queue[i].command_exists = 0;
            return err;
          }
          
  return OK;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::put_tu_in_queue (unsigned short _address, unsigned short _command) 
{
  for (unsigned int i = 0; i < tu_queue.size (); i++)
       if (tu_queue[i].command_exists == 0) {
            printf_log ("putting TU in the queue: %d address: %04x command: %02x", i, _address, _command)       
            tu_queue[i].address = _address;
            tu_queue[i].command = _command;            
            tu_queue[i].command_exists = 1;
            return OK;
          }
  printf_log ("no empty element in the TU queue, TU is eleminted");
  return OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int mek_101_device::get_TS_i_channel (unsigned short _addr, vector <mek_101_ts_buffer> &_ts_data)
{
//  print_time (); printf ("5.1\n");                                                 
  for (unsigned int i = 0; i < _ts_data.size(); i++) {  
//  print_time (); printf ("5\n");                                                 
          if ((_addr >= _ts_data[i].start_address) && (_addr <= _ts_data[i].end_address))
              return i;
      }  
  return _ts_data.size();;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int mek_101_device::get_TI_i_channel (unsigned short _addr, vector <mek_101_ti_buffer> &_ti_data)
{
  for (unsigned int i = 0; i < _ti_data.size(); i++) {  
          if ((_addr >= _ti_data[i].start_address) && (_addr <= _ti_data[i].end_address))
              return i;
      }  

  return _ti_data.size();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mek_101_device::compute_reply (vector <mek_101_ts_buffer> &_ts_data, vector <mek_101_ti_buffer> &_ti_data)
{ 
    
  unsigned int i_channel = 0;
  unsigned short i_element = 0;
  
  if (buff_in[0] == 0x68) {
       if (buff_in[6] == 1) { // TI: M_SP_NA
            if ((buff_in[7] & 0x80) == 0 ) // SQ bit 8 is 0 - N objects
                  for (unsigned int i = 0; i < buff_in[7] & 0x7f; i++) {
                         memcpy (&i_element, buff_in + 10 + i*3, 2);
//  print_time (); printf ("4.1\n");                         
                         i_channel = get_TS_i_channel (i_element, _ts_data);
//  print_time (); printf ("4.2\n");                         
                         if (i_channel >= _ts_data.size ()) {
                              printf_log ("coudn't find channel for element: %d", i_element);
                              continue;
                            }
                         i_element = i_element - _ts_data[i_channel].start_address;
                         if (i_element >= _ts_data[i_channel].ts_buffer.size()) {
                              printf_log ("too big i_element -> %d (i_channel: %d)", i_element, i_channel);
                              continue;
                            } 
                         _ts_data[i_channel].ts_buffer[i_element] = buff_in[12 + i*3];
                         printf_log ("i_channel: %d i_element: %d (%d) value: %d", i_channel, i_element, i_element + _ts_data[i_channel].start_address, _ts_data[i_channel].ts_buffer[i_element]);
                      }
            else // SQ bit 8 is 1 - 1 object, N elements
                  for (unsigned int i = 0; i < buff_in[7] & 0x7e; i++) {
            
                      }
//  print_time (); printf ("4.3\n");                                               
          } // TI: 1       
       if (buff_in[6] == 30) { // TI: M_SP_TB
            if ((buff_in[7] & 0x80) == 0 ) // SQ bit 8 is 0 - N objects
                  for (unsigned int i = 0; i < buff_in[7] & 0x7f; i++) {
                         memcpy (&i_element, buff_in + 10 + i*10, 2);
                         i_channel = get_TS_i_channel (i_element, _ts_data);
                         if (i_channel >= _ts_data.size ()) {
                              printf_log ("coudn't find channel for element: %d", i_element);                         
                              continue;
                            }
                         i_element = i_element - _ts_data[i_channel].start_address;
                         if (i_element >= _ts_data[i_channel].ts_buffer.size()) {
                              printf_log ("too big i_element -> %d (i_channel: %d)", i_element, i_channel);
                              continue;
                            }                                                      
                         _ts_data[i_channel].ts_buffer[i_element] = buff_in[12 + i*10];
                         
                         printf_log ("i_channel: %d i_element: %d (%d) value: %d", i_channel, i_element, i_element + _ts_data[i_channel].start_address, _ts_data[i_channel].ts_buffer[i_element]);
                        unsigned short msec = 0x00; 
                        memcpy (&msec, buff_in +10+10*i + 3, 2);
                        printf_log ("year: %d, month: %d day: %d clock: %d:%d:%d.%d",
                                //year                             mounth                day                    hour                   min
                                buff_in[10 + 10*i + 9] + 2000, buff_in[10+10*i + 8], buff_in[10 + 10*i + 7], buff_in[10+10*i + 6], buff_in[10 + 10*i + 5], msec/1000, msec%1000 );

                      }
            else // SQ bit 8 is 1 - 1 object, N elements
                  for (unsigned int i = 0; i < buff_in[7] & 0x7e; i++) {
            
                      }
          } // TI: 30
       if (buff_in[6] == 13) { // TI: M_ME_NC
            if ((buff_in[7] & 0x80) == 0 ) // SQ bit 8 is 0 - N objects
                  for (unsigned int i = 0; i < buff_in[7] & 0x7f; i++) {
                         memcpy (&i_element, buff_in + 10 + i*7, 2);
                         i_channel = get_TI_i_channel (i_element, _ti_data);                         
                         if (i_channel >= _ti_data.size ()) {
                              printf_log ("(TI)coudn't find channel for element: %d", i_element);
                              continue;
                            }
                         i_element = i_element - _ti_data[i_channel].start_address;
                         if (i_element >= _ti_data[i_channel].ti_buffer.size()) {
                              printf_log ("(TI)too big i_element -> %d (i_channel: %d)", i_element, i_channel);
                              continue;
                            } 
                         memcpy (&_ti_data[i_channel].ti_buffer[i_element], buff_in +12 + i*7, 4);
                         printf_log ("i_channel: %d i_element: %d (%d) value: %f", i_channel, i_element, i_element + _ti_data[i_channel].start_address, _ti_data[i_channel].ti_buffer[i_element]);
                      }
            else // SQ bit 8 is 1 - 1 object, N elements
                  for (unsigned int i = 0; i < buff_in[7] & 0x7e; i++) {
            
                      }
          } // TI: 13       
          
       if (buff_in[6] == 36) { // TI: M_ME_TF
            if ((buff_in[7] & 0x80) == 0 ) // SQ bit 8 is 0 - N objects
                  for (unsigned int i = 0; i < buff_in[7] & 0x7f; i++) {
                         memcpy (&i_element, buff_in + 10 + i*14, 2);
                         i_channel = get_TI_i_channel (i_element, _ti_data);                         
                         if (i_channel >= _ti_data.size ()) {
                              printf_log ("(TI)coudn't find channel for element: %d", i_element);
                              continue;
                            }
                         i_element = i_element - _ti_data[i_channel].start_address;
                         if (i_element >= _ti_data[i_channel].ti_buffer.size()) {
                              printf_log ("(TI)too big i_element -> %d (i_channel: %d)", i_element, i_channel);
                              continue;
                            } 
                         memcpy (&_ti_data[i_channel].ti_buffer[i_element], buff_in +12 + i*14, 4);
                         printf ("i_channel: %d i_element: %d (%d) value: %f\n", i_channel, i_element, i_element + _ti_data[i_channel].start_address, _ti_data[i_channel].ti_buffer[i_element]);
                         unsigned short msec = 0x00; 
                         memcpy (&msec, buff_in +10+14*i + 7, 2);
                         printf ("year: %d, month: %d day: %d clock: %d:%d:%d.%d\n",
                                //year                             mounth                day                    hour                   min
                                buff_in[10 + 14*i + 13] + 2000, buff_in[10+14*i + 12], buff_in[10 + 14*i + 11], buff_in[10+14*i + 10], buff_in[10 + 14*i + 9], msec/1000, msec%1000 );
                         
                      }
            else // SQ bit 8 is 1 - 1 object, N elements
                  for (unsigned int i = 0; i < buff_in[7] & 0x7e; i++) {
            
                      }
          } // TI: 36

      // preparing TU reply TI: 45
      if ((buff_in[6] == 45) && (buff_in[8] == 0x07) && (buff_in[12] == (last_tu_command | 0x80)) && (last_tu_address)) {          
           unsigned short last_tu_address_tmp = 0;
           memcpy (&last_tu_address_tmp, buff_in + 10, 2);
           if (last_tu_address == last_tu_address_tmp) 
              {            
                request_TU[12] = last_tu_command;    
                printf_log ("received preparing OK, sending exec");
                last_tu_address = 0;
                last_tu_command = 0;
                perform_tranzaction (request_TU, 15);
              }
         } // TI: 45     

     }  // 0x68
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::get_data2 (vector <unsigned char> &_ts_data)
{
  get_garbage ();
  
  while (protocol_state != ESTABLISHED) 
          set_connection ();
          

//   unsigned char request_polling[5]  = { 0x10, 0x43, 0x00, 0x00, 0x16 };
   unsigned char request_polling[15]  = { 0x68, 0x09, 0x09, 0x68, 0x73, 0x01, 0x64, 0x01, 0x06, 0x01, 0x00, 0x00, 0x14, 0x00, 0x16 };
   request_polling[5] = other_addr485; 
   request_polling[9] = COA;    
   unsigned char request_class_1_2[5]  = { 0x10, 0x5b, 0x00, 0x00, 0x16 };        
   request_class_1_2[2] = other_addr485;
                                                                                            // 94,95,96,97
   unsigned char request_TU[15]  =      { 0x68, 0x09, 0x09, 0x68, 0x73, 0x01,   45, 0x01, 0x06, 0x01, 0x94, 0x11, 0x81, 0x00, 0x16 };
   request_TU[5] = other_addr485; 
   request_TU[9] = COA;    
   
   printf_log ("activating data polling");
   if (perform_tranzaction (request_polling, 15) == ERROR) 
        return ERROR;
   request_TU[12] = 0x81;   
   int one_time_preparing = 0;   
   int one_time_executing = 0;
   int TU_latency = 0;
   send_time ();
while (1) {

/*   printf_log ("activating data polling");
   if (perform_tranzaction (request_polling, 15) == ERROR) 
        return ERROR;
*/
  
   printf_log ("request data class %c", ((request_class_1_2[1]&0x0f) == 0x0a) ? '1' : '2' );
   if (perform_tranzaction (request_class_1_2, 5) == ERROR) 
        return ERROR;
   request_class_1_2[1] = do_control_byte_function (request_class_1_2[1]);

     
  // TU 
if (TU_latency++ == 15)          
     if (one_time_preparing < 1) {
          request_TU[12] = 0x81;   
          one_time_preparing++;
          if (perform_tranzaction (request_TU, 15) == ERROR) 
               return ERROR;          
          one_time_executing = 0;
        }

     if (one_time_executing < 1) {
          request_TU[12] = 0x01;    
          if (buff_in[8] == 0x07 && buff_in[10] == 0x94 && buff_in[11] == 0x11 && buff_in[12] == 0x81 ) {
               if (perform_tranzaction (request_TU, 15) == ERROR) 
                    return ERROR;
               one_time_executing++;     
                
               one_time_preparing = 0;   
               TU_latency = 0;                
             }
        }

  // TS Job                                              <TI = 30> M_SP_TB
  if (buff_in[0] == 0x68 && buff_in[1] >= 16 && buff_in [6] == 30) {
       printf ("TS TI 30: \n");
       for (int i = 0; i < (buff_in[1] - 6)/10; i++) {
             unsigned short msec = 0x00; 
             memcpy (&msec, buff_in +10+10*i + 3, 2);
             printf ("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x (year: %d, month: %d day: %d clock: %d:%d:%d.%d)\n", buff_in[10+10*i + 0], buff_in[10 + 10*i + 1],
             buff_in[10+10*i + 2], buff_in[10 + 10*i + 3], buff_in[10+10*i + 4], buff_in[10 + 10*i + 5],
             buff_in[10+10*i + 6], buff_in[10 + 10*i + 7], buff_in[10+10*i + 8], buff_in[10 + 10*i + 9],
             //year                   mounth                day                    hour                   min
             buff_in[10 + 10*i + 9] + 2000, buff_in[10+10*i + 8], buff_in[10 + 10*i + 7], buff_in[10+10*i + 6], buff_in[10 + 10*i + 5], msec/1000, msec%1000 );
           }
     }
   usleep (500000);
}
  return OK;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char mek_101_device::do_control_byte_FCB (unsigned char _c_b)
{
   unsigned char c_b = _c_b;
   if ((c_b & 0x20)) {
       c_b = (c_b & 0x0f) | 0x50;
//printf ("2 c_b: %02x, %02x \n", c_b, (c_b & 0x50));
      }
   else {                  
       c_b = (c_b & 0x0f) | 0x70;   
//printf ("3 c_b: %02x, %02x \n", c_b, (c_b & 0x50));       
       }

 return c_b;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char mek_101_device::do_control_byte_function (unsigned char _c_b)
{ 
   unsigned char c_b = _c_b;
   if (((buff_in[1] & 0x20) && (buff_in[0] == 0x10)) || ((buff_in[4] & 0x20) && (buff_in[0] == 0x68)))
        c_b = (c_b & 0xf0) | 0x0a;
   else         
        c_b = (c_b & 0xf0) | 0x0b;

//printf ("1 c_b: %02x, %02x \n", c_b, (c_b & 0x50));
        
       
//printf ("after c_b: %02x\n", c_b);

  return c_b;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::set_connection ()
{

  if (protocol_state == ESTABLISHED) 
       return OK;   
   
   
   
   reset_tu_queue();
        
   request_polling[5] = other_addr485;    
   request_polling[9] = COA;    
   
   request_class_1_2[2] = other_addr485; 
   
   request_TU[5] = other_addr485; 
   request_TU[9] = COA;    
    
   unsigned char state_request[5]  = { 0x10, 0x49, 0x00, 0x00, 0x16 };
   state_request[2] = other_addr485; state_request[3] = checksum_256 (state_request+1, 2);
   if (perform_tranzaction (state_request, 5) == ERROR) 
        return ERROR;
         // FUNCTION - 11               PRM bit
   if (((buff_in[1] & 0x0f) != 0x0b) || ((buff_in[1] & 0x40) != 0x00)) {
        printf_log ("control byte error");
      }
                 
    
   unsigned char reset_channel[5]  = { 0x10, 0x40, 0x00, 0x00, 0x16 };
   reset_channel[2] = other_addr485; reset_channel[3] = checksum_256 (reset_channel+1, 2);
   if (perform_tranzaction (reset_channel, 5) == ERROR) 
        return ERROR;
         // FUNCTION - 0 (OK)        PRM bit
   if (((buff_in[1] & 0x0f) != 0x00) || ((buff_in[1] & 0x40) != 0x00)) {
        printf_log ("control byte error");
      }
   printf_log  ("Connection Establishment OK");      
   printf_log ("activating data polling");
   if (perform_tranzaction (request_polling, 15) == ERROR) {
        protocol_state = START;
        return ERROR;
      }
   protocol_state = ESTABLISHED;

   send_time ();
   
   return OK;
}                                                                  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::perform_tranzaction (unsigned char *_buff_out, int _size)
{
  get_garbage ();
  size_out = 0;
  size_in = 0;
//  printf ("%02x %02x\n", _buff_out[0], _buff_out[1]&0x0f);
  // don't work with FCB with starting packets ( start channel(0), start proccess(1), request channel level(9) )
  if (!((_buff_out[0] == 0x10) && (((_buff_out[1]&0x0f) == 0x00) || ((_buff_out[1]&0x0f) == 0x01) || ((_buff_out[1]&0x0f) == 0x09)))) {
      //reverse FCB  
      last_FCB ^= (1<<5);  
      if (_buff_out[0] == 0x10) { // short frame   
          _buff_out[1] = (_buff_out[1]&0x5f) | last_FCB;
         }
      else {         
            _buff_out[4] = (_buff_out[4]&0x5f) | last_FCB;   
      }
///      printf ("FCB: %02x\n", last_FCB);
    }
  
  if (_buff_out[0] == 0x10)
      _buff_out[3] = checksum_256 (_buff_out+1, 2);
  if (_buff_out[0] == 0x68)      
      _buff_out[_size-2] = checksum_256 (_buff_out+4, _size - 6);
      
   int tries = 0x00; // attempts
   while (1) {    
           if (++tries > 3)
               return ERROR;
           if (tries > 1) 
                printf_log  ("attempt: %d", tries);               
           log_bytes (_buff_out, _size, 0);  
           //  print_mek101_info (_buff_out); 
           if ((size_out = u_s->write ((char *)_buff_out, _size)) <= 0) {
                printf_log  ("can't send");
                continue;
              }
           if ((size_in = u_s->read ((char *)buff_in, 1024)) <= 0) {
                printf_log  ("can't read");
                continue;
              }     
           log_bytes (buff_in, size_in, 1);  
           if (buff_in[0] == 0x68) {      
                if (Check_FT1_2_variable (buff_in, size_in) == ERROR)
                     continue;
              }
           else {
                 if (buff_in[0] == 0x10) 
                      if (Check_FT1_2_fixed (buff_in, size_in) == ERROR)
                          continue;
           }
           break;
         }
                          
 return OK;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::send_time ()
{
   printf_log ("sending time");                           // 0x44                                       // msec      min   hours day   mounth year
   unsigned char time_sync[21] = { 0x68, 0x0f, 0x0f, 0x68, 0x53, 0x64, 0x67, 0x01, 0x06, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x16 };
   time_sync[5] = other_addr485; 
   time_sync[9] = COA;        
   tm  time_local; 
   timeval t1;
   gettimeofday (&t1, NULL);
   memcpy (&time_local, localtime (&t1.tv_sec), sizeof (tm));
//   memcpy (&time_local, gmtime (&t1.tv_sec), sizeof (tm));
   printf_log ("sync time -> year: %d: month: %d day: %d week day: %d hour: %d min: %d sec %d usec %ld",
            time_local.tm_year, time_local.tm_mon + 1, time_local.tm_mday, time_local.tm_wday+1, time_local.tm_hour, 
            time_local.tm_min, time_local.tm_sec, t1.tv_usec);
   // need to send in GM Time without local
   memcpy (&time_local, gmtime (&t1.tv_sec), sizeof (tm));            
   unsigned char time_buffer [7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   ushort msec = time_local.tm_sec * 1000 + t1.tv_usec / 1000;
   memcpy (time_buffer, &msec, 2);
   time_buffer [2] = time_local.tm_min;
   time_buffer [3] = time_local.tm_hour;
//   time_buffer [4] = ((time_local.tm_wday+1)<<5)|time_local.tm_mday;	 
   time_buffer [4] = time_local.tm_mday;	 
   time_buffer [5] = time_local.tm_mon + 1;		 
   time_buffer [6] = time_local.tm_year - 100;         
   for (int i  = 0; i < 7; i++)
         time_sync [12 + i] = time_buffer[i];
   time_sync[21-2] = checksum_256 (time_sync+4, 21 - 6);         

   if (perform_tranzaction (time_sync, 21) == ERROR) 
        return ERROR;

/*        
   printf ("->");
   print_bytes (time_sync, 21);                      
   if ((size_out = u_s->write ((char *)time_sync, 21)) <= 0) {
        printf_log  ("can't send");
        return ERROR;
      }
*/

   return OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mek_101_device::get_garbage ()
{
  u_s->set_read_timeout (0, 0);   
  size_in = 0;
  while ((size_in = u_s->read ((char *)buff_in, 1024)) > 0) {
          printf_log2 ("garbage(size: %d): ", size_in);
          log_bytes (buff_in, size_in, 1);
        }       
  u_s->set_read_timeout (timeout_first_byte, timeout_next_byte);        
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	    
mek_101_device::~mek_101_device ()
{
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mek_101_device::log_bytes (unsigned char *buffer, int _size, int _in_out) 
{
 
   char str_log [1024];
  int j = 0;
  if (_size > 330) {
      printf_log ("truncated to 330 bytes...")
      _size = 330;
     }
  for (int i = 0; i < _size; i++ ) {
        sprintf (str_log+j, " %02x", buffer[i]);
	j = j + 3;
      }      
  str_log[j] = '\0';
  printf_log ("%s %s", _in_out ? "<-": "->", str_log);  
};
///////////////////////////////////////////////
void mek_101_device::print_bytes (unsigned char *buffer, int _size) 
{
  for (int i = 0, j = 0; i < _size; i++ ) {
        printf (" %02x", buffer[i]);
	j = j + 3;
      }      
  printf ("\n");    
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::Check_FT1_2_variable (unsigned char *_buf, int _size)
{
  //FT1_2_HEAD * h = (FT1_2_HEAD *) _buf;
//printf ("1 %d\n", _size);  
  if (_buf[0] != START_WORD) {
       printf_log ("leading byte is not 0x68: %d", _buf[0]);
       
       return ERROR;
     }
//printf ("2\n");
  if (_buf[1] != _buf[2]) {
       printf_log ("second bytes and third bytes are not the same %d %d", _buf[1], _buf[2]);
       return ERROR;
     }

  if (_buf[1] + 6 != _size) {
       printf_log ("size in packet + 6: %d is not rec size: %d", _buf[1] + 6, _size);       
       return ERROR;
     }

//printf ("3\n");
  if (_buf[3] != START_WORD) {
//       msg_obj->log ("forth byte is not 0x68");
       printf_log ("forth byte is not 0x68: %02x", _buf[3]);
       return ERROR;
     }
//printf ("4\n");
  if (_buf[_size-1] != END_WORD) {
       printf_log ("end byte is not 0x16 %d", _buf[_size-1]);
       return ERROR;
     }
//printf ("5\n");
  if (_buf[_size-2] != checksum_256 (_buf + 4, _size - 6)) {
       printf_log ("checksum missmatch calc: 0x%02x received: 0x%02x",checksum_256 (_buf + 4, _size - 6),  _buf[_size-2]);
       return ERROR;                    
     }
//printf ("6\n");
  return OK;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::Check_FT1_2_fixed (unsigned char *_buf, int _size) 
{
  if (_buf[0] != 0x10) {
       printf_log ("leading byte is not 0x10: %d (fixed)", _buf[0]);       
       return ERROR;
     }
  if (_buf[_size-1] != 0x16) {
       printf_log ("end byte is not 0x16 %d(fixed)", _buf[_size-1]);
       return ERROR;
     }
  if (_buf[_size-2] != checksum_256 (_buf + 1, 2)) {
       printf_log ("checksum missmatch calc: 0x%02x received: 0x%02x(fixed)",checksum_256 (_buf + 1, 2),  _buf[_size-2]);
       return ERROR;                    
     }
  return OK;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mek_101_device::print_mek101_info (const unsigned char * _buff) 
{ 
  char str[256];
  if (_buff[0] == 0x10) { // short packet
       if ((_buff[1]&0x40)>>6) { // primary station
            sprintf (str, "PRM: %d FCB: %d FCV: %d FUNC: %02d A: %02d", (_buff[1]&0x40)>>6, (_buff[1]&0x20)>>5, (_buff[1]&0x10)>>4, _buff[1]&0x0f, _buff[2]);
          }
       else {
              sprintf (str, "PRM: %d ACD: %d DFC: %d FUNC: %02d A: %02d", (_buff[1]&0x40)>>6, (_buff[1]&0x20)>>5, (_buff[1]&0x10)>>4, _buff[1]&0x0f, _buff[2]);
       }     
     }
  else {
        if ((_buff[4]&0x40)>>6) { // primary station
             sprintf (str, "PRM: %d FCB: %d FCV: %d FUNC: %02d A: %02d", (_buff[4]&0x40)>>6, (_buff[4]&0x20)>>5, (_buff[4]&0x10)>>4, _buff[4]&0x0f, _buff[5]);
           }
        else {
              sprintf (str, "PRM: %d ACD: %d DFC: %d FUNC: %02d A: %02d", (_buff[4]&0x40)>>6, (_buff[4]&0x20)>>5, (_buff[4]&0x10)>>4, _buff[4]&0x0f, _buff[5]);
        }     
  }     
  printf ("%s\n", str);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
STATUS mek_101_device::isActive () 
{
   FILE   * FprocNet;
   if ((FprocNet = fopen ("/tmp/lpstatus", "r")) == NULL) {
       perror ("fopen /tmp/lpstatus error");
 //      fclose (FprocNet);  // came from Otrada MSK ZHD, Dialog Connectivity. Fixed on Komendanyski pr. STPV-15.
       return ERROR;
       }       
   int state = 0;
   fscanf (FprocNet, "%d\n", &state); 
   fclose (FprocNet); 
   if (state == 1) 
       return OK;
   else 
       return ERROR;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char mek_101_device::checksum_256 (unsigned char *_buf, unsigned char _size)
{
  unsigned short crc_256 = 0x00;
  for (unsigned char i = 0; i < _size; i++) {
        crc_256 = crc_256 + _buf[i];
      } 
  unsigned char *z = (unsigned char *)&crc_256;
  return (unsigned char) *z;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
