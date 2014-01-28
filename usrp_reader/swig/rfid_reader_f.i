GR_SWIG_BLOCK_MAGIC(rfid, reader_f);

rfid_reader_f_sptr
rfid_make_reader_f(int sample_rate);

class rfid_reader_f: public gr_block{
  rfid_reader_f(int sample_rate);

public:
  ~rfid_reader_f();
  gr_msg_queue_sptr    ctrl_q();
  gr_msg_queue_sptr get_log();
  gr_msg_queue_sptr get_tag_q();
  bool check_reader_state();
};
