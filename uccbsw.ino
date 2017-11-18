/*
#define SW10P_1      92
#define SW10P_2      188
#define SW10P_3      283
#define SW10P_4      379
#define SW10P_5      476
#define SW10P_6      570
#define SW10P_7      666
#define SW10P_8      762
#define SW10P_9      857
#define SW10P_10     953
*/

#define SW10P_1      92
#define SW10P_2      188
#define SW10P_3      283
#define SW10P_4      377
#define SW10P_5      472
#define SW10P_6      567
#define SW10P_7      663
#define SW10P_8      758
#define SW10P_9      853
#define SW10P_10     947

#define SW10P_GAP    20

int sw_setup(void)
{
  int rval=-1;
  
  smar_init(&g_smar_sw10p,SMAR_ADCLOC_ARDUINO,UCCB_SW10P_PORT,7,3,6);
  smar_init(&g_smar_b6p,SMAR_ADCLOC_ARDUINO,UCCB_B6P_PORT,3,3,6);
  
  uccbbtn_init(&g_btn_b6p,UCCB_B6P_PORT,1);
  
  tmr_init(&g_tmr_sw10p,25);
  tmr_init(&g_tmr_b6p,5);
  
  rval=0;
  
end:

  return(rval);
}

int sw10p_readA(void)
{
  return(smar_analogRead(&g_smar_sw10p));
}

int sw10p_readP(void)
{
  static int l_pos=-1;
//  static unsigned long l_sw10prt=0;
  
  if(tmr_do(&g_tmr_sw10p) != 1) return(l_pos);
//  if((millis()-l_sw10prt) < 25) return(l_pos);
  
  l_pos=smar_analogRead(&g_smar_sw10p);
  if((l_pos > SW10P_1-SW10P_GAP) && (l_pos < SW10P_1+SW10P_GAP)) l_pos=1;
  else if((l_pos > SW10P_2-SW10P_GAP) && (l_pos < SW10P_2+SW10P_GAP)) l_pos=2;
  else if((l_pos > SW10P_3-SW10P_GAP) && (l_pos < SW10P_3+SW10P_GAP)) l_pos=3;
  else if((l_pos > SW10P_4-SW10P_GAP) && (l_pos < SW10P_4+SW10P_GAP)) l_pos=4;
  else if((l_pos > SW10P_5-SW10P_GAP) && (l_pos < SW10P_5+SW10P_GAP)) l_pos=5;
  else if((l_pos > SW10P_6-SW10P_GAP) && (l_pos < SW10P_6+SW10P_GAP)) l_pos=6;
  else if((l_pos > SW10P_7-SW10P_GAP) && (l_pos < SW10P_7+SW10P_GAP)) l_pos=7;
  else if((l_pos > SW10P_8-SW10P_GAP) && (l_pos < SW10P_8+SW10P_GAP)) l_pos=8;
  else if((l_pos > SW10P_9-SW10P_GAP) && (l_pos < SW10P_9+SW10P_GAP)) l_pos=9;
  else if((l_pos > SW10P_10-SW10P_GAP) && (l_pos < SW10P_10+SW10P_GAP)) l_pos=10;
  else l_pos=0;
  
//  l_sw10prt=millis();
  
  return(l_pos);
}

int b6p_readA(void)
{
  return(smar_analogRead(&g_smar_b6p));
}

int b6p_readP(void)
{
  int pos;
  static int lpos=0,lpos_ret=0;
  static byte lpos_cnt=0;
//  static unsigned long l_b6pprt=0;

  if(tmr_do(&g_tmr_b6p) != 1) return(lpos_ret);
//  if((millis()-l_b6pprt) < 5) return(lpos_ret);
  
  pos=smar_analogRead(&g_smar_b6p);
  
//  Serial.println(pos);
  
  if((pos > SW10P_1-SW10P_GAP) && (pos < SW10P_1+SW10P_GAP)) pos=1;
  else if((pos > SW10P_2-SW10P_GAP) && (pos < SW10P_2+SW10P_GAP)) pos=2;
  else if((pos > SW10P_3-SW10P_GAP) && (pos < SW10P_3+SW10P_GAP)) pos=3;
  else if((pos > SW10P_4-SW10P_GAP) && (pos < SW10P_4+SW10P_GAP)) pos=4;
  else if((pos > SW10P_5-SW10P_GAP) && (pos < SW10P_5+SW10P_GAP)) pos=5;
  else if((pos > SW10P_6-SW10P_GAP) && (pos < SW10P_6+SW10P_GAP)) pos=6;
  else pos=0;
  
  if(pos > 0) {
    if(lpos == pos) {
      if(lpos_cnt > 5) {
      } else {
        pos=0;
        lpos_cnt++;
      }
    } else {
      lpos_cnt=1;
      lpos=pos;
      pos=0;
    }
  }
    
//  l_b6pprt=millis();

  lpos_ret=pos;
  return(pos);
}

int b6p_btn(int *be)
{
  int bs;
  
  bs=uccb_btn_check(&g_btn_b6p,be);
  
  return(bs);
}

