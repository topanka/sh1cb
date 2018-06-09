#define KP_ROWS  4
#define KP_COLS  3
char g_keys[KP_ROWS][KP_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
//31(PC6), 33(PC4)
byte rowPins[KP_ROWS] = {31, 33, 22, 23}; //connect to the row pinouts of the keypad
byte colPins[KP_COLS] = {24, 25, 26}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(g_keys), rowPins, colPins, KP_ROWS, KP_COLS);

int keyp_setup(void)
{
  g_key=NO_KEY;

  return(0);
}

int keyp_read(char *key)
{
  if((g_sw10p != 1) && 
     (g_sw10p != 5) &&
     (g_sw10p != 6) &&
     (g_sw10p != 8)) {
    *key='\0';
    return(0);
  }
  *key=keypad.getKey();
  if(*key == NO_KEY) {
    *key='\0';
    return(0);
  }
  return(1);
}

