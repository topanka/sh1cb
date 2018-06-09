#define BTN_NOP               0        //no operation
#define BTN_HOLD              1        //hold
#define BTN_RFH               2        //release from hold
#define BTN_CLK_BASE          10       //click

/*
#define BTN_LOW_TMO           200
#define BTN_HIGH_TMO          500
*/
#define BTN_LOW_TMO           200
#define BTN_HIGH_TMO          600


int uccbbtn_setup(void)
{
  return(0);
}

int uccbbtn_init(UCCBBTN *btn, uint8_t port, int nlg)
{
  int rval=-1;
  
  if(btn == NULL) goto end;
  btn->port=port;
  btn->nlg=nlg;
  btn->bsl_s=LOW;

  rval=0;

end:  

  return(rval);
}

int uccb_btn_check(UCCBBTN *btn, int *ev)
{
  int bs=-1;

  if(btn->nlg == 1) {
    bs=b6p_readP();
  } else {
    bs=digitalRead(btn->port);
  }
  
  if(bs > btn->last_not_low) {
    btn->last_not_low=bs;
  }

  if(bs == btn->bsl_s) {
//    if(bs == HIGH) {
    if(bs != LOW) {
      if((g_millis-btn->bsl_t) > BTN_HIGH_TMO) {
         btn->hop=1;
         *ev=BTN_HOLD;
         return(bs);
      }
      *ev=BTN_NOP;
      return(bs);
    }
    if(btn->bsl_t == 0) {
      *ev=BTN_NOP;
       return(bs);
    }
    
//  Serial.println(now-bsl_t);
//  Serial.println(clkn);
  
    if((g_millis-btn->bsl_t) > BTN_LOW_TMO) {
//      *ev=btn->clkn+BTN_CLK_BASE;
      *ev=btn->clkn+BTN_CLK_BASE*btn->last_not_low;
      btn->clkn=0;
      btn->bsl_t=0;
      btn->last_not_low=0;
      return(bs);
    }
    *ev=BTN_NOP;
    return(bs);
  }
  if(btn->bsl_t == 0) {
    *ev=BTN_NOP;
    btn->bsl_s=bs;
    btn->bsl_t=g_millis;
    return(bs);
  }
  if(bs == LOW) {
    if((g_millis-btn->bsl_t) > BTN_HIGH_TMO) {
      if(btn->hop == 1) {
        *ev=BTN_RFH;
        btn->hop=0;
      } else {
//        *ev=btn->clkn+BTN_CLK_BASE;
        *ev=btn->clkn+BTN_CLK_BASE*btn->last_not_low;
      }
      btn->clkn=0;
      btn->bsl_s=bs;
      btn->bsl_t=0;
      btn->last_not_low=0;
      return(bs);
    }
    btn->clkn++;
//    btn->last_not_low=btn->bsl_s;
  } else {
  }
  *ev=BTN_NOP;
  btn->bsl_s=bs;
//  bsl_t=now;

end:

  return(bs);
}

