void loop_counter(void)
{
  g_loop_cnt++;
  if((g_millis-g_loop_ct) > 1000) {
    g_loop_cps=g_loop_cnt;
    g_loop_cnt=0;
    g_loop_ct=g_millis;
  }
}

void setup()
{
  buildCRCTable();
  adc_setup();
  dsp_setup();
  smar_setup();
  uccbbtn_setup();
  batt_setup();
  finscan_setup();
  joy_setup();
  sw_setup();
  keyp_setup();
  comm_setup();
  
  Serial.begin(115200);
  
  dsp_scr_startup();
  uccb_clb_load();
  tscr_setup();
  delay(1000);

  uccb_clb_autocenter_TS();
  uccb_clb_autocenter_FS();
  eval_setup();
  
  g_loop_ct=millis();
}

void loop()
{
  g_millis=millis();
  loop_counter();
  batt_read();

  joy_ts_read(&g_tsX,&g_tsY);
  joy_fs_read(&g_fsX,&g_fsY,&g_fsZ);
   
  g_fsBS=joy_fs_btn(&g_fsBE);
//g_fsBE=BTN_NOP;

  g_sw10p=sw10p_readP();
//g_sw10p=1;

//  g_sw10p=sw10p_readA();
//  g_b6p=b6p_readP();
//    g_b6p=b6p_readA();
//    g_b6p=analogRead(UCCB_B6P_PORT);

  g_b6pBS=b6p_btn(&g_b6pBE);
//g_b6pBE=BTN_NOP;

  keyp_read(&g_key);

  comm_send();
  comm_recv();

//  comm_test();
  
  dsp_print();

  batt_poweroff();

  tscr_comm();
  
}

