//GPIO0-SUCCESS-  43(PL6)
//GPIO1-FAIL-     42(PL7)
//GPIO2-REG-      37(PC0)
//GPIO3-DEL-      36(PC1)
//GPIO4-IDENT-    35(PC2)

#define FINSCAN_SERVO_MIN_POS            1400
#define FINSCAN_SERVO_MAX_POS            2085
#define FINSCAN_SERVO_STEP               (2100-FINSCAN_SERVO_MAX_POS)

typedef struct tagMYAVRTIMER {
  volatile uint8_t *tccr;
  volatile uint16_t *icr;
  volatile uint16_t *ocr;
} MYAVRTIMER;

int g_finscan_op_onprogress=0;
int g_finscan_op_completed=1;
unsigned long g_finscan_hold_time=0;
unsigned long g_finscan_do_time=0;
int g_finscan_do_port=0;
int g_finscan_op_succ=0;
int g_finscan_op_fail=0;
unsigned long g_finscan_op_time=0;

void timer_test(void)
{
  MYAVRTIMER mat;

  TCCR1A = 0b10100000; // Mega pin 5
  TCCR1B = 0b00010001; // Mega pin 3
  ICR1 = 400;
  OCR1A = 250;
  OCR1B = 250;

  TCCR4A = 0b10100000; // Mega pin 5
  TCCR4B = 0b00010001; // Mega pin 3
  TCCR4C = 0b00010001; // Mega pin 3
  ICR4 = 400;
  OCR4A = 250;
  OCR4B = 250;
  OCR4C = 250;

  TCCR5A = 0b10100000; // Mega pin 5
  TCCR5B = 0b00010001; // Mega pin 3
  TCCR5C = 0b00010001; // Mega pin 3
  ICR5 = 400;
  OCR5A = 250;
  OCR5B = 250;
  OCR5C = 250;

  mat.tccr=&TCCR4A;
  mat.icr=&ICR4;
  mat.ocr=&OCR4C;
  
}

void finscan_setup(void)
{
  ump_pinMode(UCCB_FS_SRV_PWR_UMPPORT,OUTPUT);
//  pinMode(UCCB_FS_SRV_PWR_PORT,OUTPUT);
//  ump_digitalWrite(UCCB_FS_SRV_PWR_UMPPORT,UCCB_FS_SRV_PWR_OFF);
  digitalWrite(UCCB_FS_SRV_PWR_UMPPORT,UCCB_FS_SRV_PWR_OFF);
  g_fs_servo_on=0;
  
  g_servo_finscan.attach(7,900,2100);
  g_servo_finscan.writeMicroseconds(FINSCAN_SERVO_MIN_POS);
  tmr_init(&g_tmr_finscan,20);
  pinMode(FIM5360_SUCC_PORT,INPUT_PULLUP);
  pinMode(FIM5360_FAIL_PORT,INPUT_PULLUP);
  pinMode(FIM5360_REG_PORT,OUTPUT);
  pinMode(FIM5360_DEL_PORT,OUTPUT);
  pinMode(FIM5360_IDENT_PORT,OUTPUT);
  digitalWrite(FIM5360_REG_PORT,HIGH);
  digitalWrite(FIM5360_DEL_PORT,HIGH);
  digitalWrite(FIM5360_IDENT_PORT,HIGH);
}

void finscan_move(int show)
{
  if(tmr_do(&g_tmr_finscan) == 1) {
    if((g_finscan_pos < FINSCAN_SERVO_MAX_POS) && (g_finscan_pos > FINSCAN_SERVO_MIN_POS)) {
      if(g_fs_servo_on == 0) {
        ump_digitalWrite(UCCB_FS_SRV_PWR_UMPPORT,UCCB_FS_SRV_PWR_ON);
//        digitalWrite(UCCB_FS_SRV_PWR_PORT,UCCB_FS_SRV_PWR_ON);
        g_fs_servo_on=1;
      }
    }
    if((g_finscan_pos < FINSCAN_SERVO_MAX_POS) || (g_finscan_pos > FINSCAN_SERVO_MIN_POS)) {
      if(show == 1) {
        if(g_finscan_pos < FINSCAN_SERVO_MAX_POS) g_finscan_pos+=FINSCAN_SERVO_STEP;
      } else if(show == 0) {
        if(g_finscan_pos > FINSCAN_SERVO_MIN_POS) g_finscan_pos-=FINSCAN_SERVO_STEP;
      }
      g_servo_finscan.writeMicroseconds(g_finscan_pos);
    }
    if(g_finscan_pos <= FINSCAN_SERVO_MIN_POS) {
      if(g_fs_servo_on == 1) {
        delay(150);
        ump_digitalWrite(UCCB_FS_SRV_PWR_UMPPORT,UCCB_FS_SRV_PWR_OFF);
//        digitalWrite(UCCB_FS_SRV_PWR_PORT,UCCB_FS_SRV_PWR_OFF);
        g_fs_servo_on=0;
      }
    }
  }
}

int finscan_op(unsigned int op_port)
{
  finscan_move(1);

  if((g_finscan_pos >= FINSCAN_SERVO_MAX_POS) &&
     (g_finscan_op_onprogress == 0)) {
    g_finscan_op_onprogress=1;
    g_finscan_op_completed=0;
    g_finscan_hold_time=50;
    digitalWrite(op_port,LOW);
    g_finscan_do_time=g_millis;
    g_finscan_do_port=op_port;
  }
  if(g_finscan_do_port != 0) {
    if(g_finscan_do_time+g_finscan_hold_time <= g_millis) {
      digitalWrite(g_finscan_do_port,HIGH);
      g_finscan_do_port=0;
      g_finscan_do_time=0;
    }
  }
  g_finscan_op_succ=digitalRead(FIM5360_SUCC_PORT);
  g_finscan_op_fail=digitalRead(FIM5360_FAIL_PORT);
  if(g_finscan_op_succ == HIGH) {
    if(g_finscan_op_time == 0) {
//      Serial.println("SUCCESS");
      g_finscan_op_time=g_millis;
      g_finscan_op_completed=1;
      g_finscan_op_onprogress=0;
      return(1);
    } else {
      if(g_finscan_op_time+1100 < g_millis) {
        g_finscan_op_time=0;
      }
    }
  }
  if(g_finscan_op_fail == HIGH) {
    if(g_finscan_op_time == 0) {
//      Serial.println("FAIL");
      g_finscan_op_time=g_millis;
      g_finscan_op_completed=1;
      g_finscan_op_onprogress=0;
      return(-1);
    } else {
      if(g_finscan_op_time+1100 < g_millis) {
        g_finscan_op_time=0;
      }
    }
  }
  return(0);
}

int finscan_cancel(void)
{
  g_finscan_op_onprogress=0;
}

