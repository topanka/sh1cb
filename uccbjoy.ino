int l_joy_bsl=-1,l_joy_dbl=0;
unsigned long l_joy_bslt=LOW,l_joy_r2pt=0,l_joy_p2rt=0;

int joy_setup(void)
{
  int rval=-1;

//  smar_init(&g_smar_tsX,SMAR_ADCLOC_ARDUINO,UCCB_TS_X,5,3);
//  smar_init(&g_smar_tsY,SMAR_ADCLOC_ARDUINO,UCCB_TS_Y,5,3);
  smar_init(&g_smar_tsX,SMAR_ADCLOC_MCP3008,UCCB_TS_X,7,3,5);
  smar_init(&g_smar_tsY,SMAR_ADCLOC_MCP3008,UCCB_TS_Y,7,3,5);
  
//  smar_init(&g_smar_fsX,SMAR_ADCLOC_ARDUINO,UCCB_FS_X,7,3);
//  smar_init(&g_smar_fsY,SMAR_ADCLOC_ARDUINO,UCCB_FS_Y,7,3);
//  smar_init(&g_smar_fsZ,SMAR_ADCLOC_ARDUINO,UCCB_FS_Z,7,3);
  
  smar_init(&g_smar_fsX,SMAR_ADCLOC_MCP3008,UCCB_FS_X,9,3,6);
  smar_init(&g_smar_fsY,SMAR_ADCLOC_MCP3008,UCCB_FS_Y,9,3,6);
  smar_init(&g_smar_fsZ,SMAR_ADCLOC_MCP3008,UCCB_FS_Z,9,3,6);

  pinMode(UCCB_FS_BTN,INPUT);     
//  digitalWrite(UCCB_FS_BTN,HIGH);

  uccbbtn_init(&g_btn_fs,UCCB_FS_BTN,0);
 
  rval=0;
  
end:

  return(rval);
}

int joy_ts_read(int *x, int *y)
{
  int x1;

/*
  x1=adc_single_channel_read(mcp3008_ch0);
  ts_x.addValue(x1);
  *x=ts_x.getAverage();
*/  

  *x=smar_analogRead(&g_smar_tsX);
  *y=smar_analogRead(&g_smar_tsY);

  return(0);
}

int joy_fs_read(int *x, int *y, int *z)
{
  *x=smar_analogRead(&g_smar_fsX);
  *y=smar_analogRead(&g_smar_fsY);
  *z=smar_analogRead(&g_smar_fsZ);
  return(0);
}

int joy_fs_btn(int *be)
{
  int bs;
  
  bs=uccb_btn_check(&g_btn_fs,be);
  
  return(bs);
}

