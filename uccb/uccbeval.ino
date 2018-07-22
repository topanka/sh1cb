#define UCCB_MD_MINSPEED_FWD      55
#define xUCCB_MD_MAXSPEED_FWD      400
#define UCCB_MD_MINSPEED_REW      -55
#define xUCCB_MD_MAXSPEED_REW      -400

/*
#define UCCB_MD_M1_MINSPEED_FWD      45
#define UCCB_MD_M1_MAXSPEED_FWD      400
#define UCCB_MD_M1_MINSPEED_REW      -45
#define UCCB_MD_M1_MAXSPEED_REW      -400
*/

//#define UCCB_FS_CENGAP            6
//#define UCCB_TS_CENGAP            6

#define UCCB_FS_CENGAP            7
#define UCCB_TS_CENGAP            7

#define UCCB_FS_EDGEGAP_X         12
#define UCCB_FS_EDGEGAP_Y         2
#define UCCB_FS_EDGEGAP_Z         5

int g_md_maxspeed_fwd;
int g_md_maxspeed_rew;

int g_ts_x_LEFT;
int g_ts_y_FWD;
int g_ts_x_RIGHT;
int g_ts_y_REW;

int g_fs_DC_LEFT;					//direction change on left side
int g_fs_DC_RIGHT;					//direction change on right side
int g_fs_x_LEFT;
int g_fs_y_FWD;
int g_fs_z_CW;
int g_fs_x_RIGHT;
int g_fs_y_REW;
int g_fs_z_CCW;

int fsqrt(int x1, int x2)
{
  unsigned long root = 0;
  unsigned long bit = 1UL << 30;
  unsigned long number;
 
  number=(unsigned long)x1*(unsigned long)x1+(unsigned long)x2*(unsigned long)x2;
  // Bit starts at the highest power of four <= to input number.
  while(bit > number)  bit >>= 2;
 
  while(bit != 0){
    if(number >= root + bit){
        number -= (root + bit);
        root += (bit << 1);
    }
    root >>= 1;
    bit >>= 2;
  }
 
  return((int)root);
}

int iscale( int originalMin, int originalMax, int newBegin, int newEnd, int inputValue, float curve){

  int OriginalRange=0;
  int NewRange=0;
  int zeroRefCurVal=0;
  float normalizedCurVal=0;
  int rangedValue=0;

  // condition curve parameter
  // limit range

/*
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
*/  

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if(inputValue < originalMin) {
    inputValue=originalMin;
  }
  if(inputValue > originalMax) {
    inputValue=originalMax;
  }

  // Zero Refference the values
  OriginalRange=originalMax-originalMin;

  zeroRefCurVal=inputValue-originalMin;
  normalizedCurVal=(float)zeroRefCurVal/OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if(originalMin > originalMax ) {
    return(0);
  }

  if(newEnd > newBegin) {
    NewRange=newEnd-newBegin;
    rangedValue=(int)(pow(normalizedCurVal,curve)*NewRange)+newBegin;
  } else {
    NewRange=newBegin-newEnd;
    rangedValue=newBegin-(int)(pow(normalizedCurVal,curve)*NewRange);
  }

  return(rangedValue);
}

int eval_setmaxspeed(void)
{
  g_md_maxspeed_fwd=g_sh1_maxmspeed;
  g_md_maxspeed_rew=-g_sh1_maxmspeed;
  
  return(0);
}

int eval_setup(void)
{
  g_ts_x_LEFT=g_ts_maxX-g_ts_cenX;
  g_ts_y_FWD=g_ts_cenY-g_ts_minY;
  g_ts_x_RIGHT=g_ts_cenX-g_ts_minX;
  g_ts_y_REW=g_ts_maxY-g_ts_cenY;
  
  g_fs_DC_LEFT=(3*(g_fs_maxX-g_fs_cenX))/4;
  g_fs_DC_RIGHT=(3*(g_fs_cenX-g_fs_minX))/4;

  g_fs_x_LEFT=g_fs_maxX-g_fs_cenX-UCCB_FS_EDGEGAP_X-UCCB_FS_CENGAP;
  g_fs_y_FWD=g_fs_cenY-g_fs_minY-UCCB_FS_EDGEGAP_Y-UCCB_FS_CENGAP;
  g_fs_z_CW=g_fs_cenZ-g_fs_minZ-UCCB_FS_EDGEGAP_Z-UCCB_FS_CENGAP;
  g_fs_x_RIGHT=g_fs_cenX-g_fs_minX-UCCB_FS_EDGEGAP_X-UCCB_FS_CENGAP;
  g_fs_y_REW=g_fs_maxY-g_fs_cenY-UCCB_FS_EDGEGAP_Y-UCCB_FS_CENGAP;
  g_fs_z_CCW=g_fs_maxZ-g_fs_cenZ-UCCB_FS_EDGEGAP_Z-UCCB_FS_CENGAP;

  eval_setmaxspeed();
  
  return(0);
}

int eval_md_speed(int *m1s, int *m2s, int *rdd)
{
  long vx,vy,vz,v1,v2,q,Mx,My,vxn,vyn,vzn;
  int rd0;

  vx=constrain(g_fsX,g_fs_minX,g_fs_maxX);
  vy=constrain(g_fsY,g_fs_minY,g_fs_maxY);
  vz=constrain(g_fsZ,g_fs_minZ,g_fs_maxZ);

  vx=g_fs_cenX-vx;
  vy=g_fs_cenY-vy;
  vz=g_fs_cenZ-vz;
  
  if(vx > 0) {
    vx=constrain(vx,0,g_fs_x_RIGHT+UCCB_FS_CENGAP);
  } else if(vx < 0) {
    vx=constrain(vx,-g_fs_x_LEFT-UCCB_FS_CENGAP,0);
  }
  if(vy > 0) {
    vy=constrain(vy,0,g_fs_y_FWD+UCCB_FS_CENGAP);
  } else if(vy < 0) {
    vy=constrain(vy,-g_fs_y_REW-UCCB_FS_CENGAP,0);
  }
  if(vz > 0) {
    vz=constrain(vz,0,g_fs_z_CW+UCCB_FS_CENGAP);
  } else if(vz < 0) {
    vz=constrain(vz,-g_fs_z_CCW-UCCB_FS_CENGAP,0);
  }
  
  vxn=abs(vx);
  vyn=abs(vy);
  vzn=abs(vz);

  if(vxn <= UCCB_FS_CENGAP) {
    vx=0;
    vxn=0;
  } else {
    if(vx > 0) vx-=UCCB_FS_CENGAP;
    else vx+=UCCB_FS_CENGAP;
  }
  if(vyn <= UCCB_FS_CENGAP) {
    vy=0;
    vyn=0;
  } else {
    if(vy > 0) vy-=UCCB_FS_CENGAP;
    else vy+=UCCB_FS_CENGAP;
  }
  if(vzn <= UCCB_FS_CENGAP) {
    vz=0;
    vzn=0;
  } else {
    if(vz > 0) vz-=UCCB_FS_CENGAP;
    else vz+=UCCB_FS_CENGAP;
  }

  if((vx == 0) && (vy == 0) && (vz != 0)) {
    if(vz > 0) {
      v1=map(vzn,0,g_fs_z_CW,UCCB_MD_MINSPEED_FWD,g_md_maxspeed_fwd);
    } else {
      v1=map(vzn,0,g_fs_z_CCW,UCCB_MD_MINSPEED_REW,g_md_maxspeed_rew);
    }
    v1=abs(v1);
    vx=-vz;
  } else {
    q=0;
    if((vx == 0) && (vy == 0)) {
      v1=0;
    } else if(vy == 0) {
      if(vx > 0) {
        v1=map(vx,0,g_fs_x_RIGHT,UCCB_MD_MINSPEED_FWD,xUCCB_MD_MAXSPEED_FWD/4);
      } else {
        v1=map(vxn,0,g_fs_x_LEFT,UCCB_MD_MINSPEED_FWD,xUCCB_MD_MAXSPEED_FWD/4);
      }
    } else if(vx == 0) {
      if(vy > 0) {
  //      v1=map(vy,0,g_fs_y_FWD,UCCB_MD_MINSPEED_FWD,UCCB_MD_MAXSPEED_FWD);
        v1=map(vy,0,g_fs_y_FWD,UCCB_MD_MINSPEED_FWD,g_md_maxspeed_fwd);
      } else {
  //      v1=map(vy,0,-g_fs_y_REW,UCCB_MD_MINSPEED_REW,UCCB_MD_MAXSPEED_REW);
        v1=map(vy,0,-g_fs_y_REW,UCCB_MD_MINSPEED_REW,g_md_maxspeed_rew);
      }
    } else if((vx > 0) && (vy > 0)) {
      q=1;
      Mx=g_fs_x_RIGHT;
      My=g_fs_y_FWD;
    } else if((vx < 0) && (vy > 0)) {
      q=2;
      Mx=g_fs_x_LEFT;
      My=g_fs_y_FWD;
    } else if((vx < 0) && (vy < 0)) {
      q=3;
      Mx=g_fs_x_LEFT;
      My=g_fs_y_REW;
    } else if((vx > 0) && (vy < 0)) {
      q=4;
      Mx=g_fs_x_RIGHT;
      My=g_fs_y_REW;
    } else {
      v1=0;
    }
  
  /*
    Serial.print(Mx);
    Serial.print(" ");
    Serial.print(My);
    Serial.print(" ");
    Serial.print(R);
    Serial.print(" ");
    Serial.println(q);
  */
  
    if(q != 0) {
      if(vy >= 0) {
  //      v1=map(vy,0,My,UCCB_MD_MINSPEED_FWD,UCCB_MD_MAXSPEED_FWD);
        v1=map(vy,0,My,UCCB_MD_MINSPEED_FWD,g_md_maxspeed_fwd);
      } else {
  //      v1=map(vy,0,-My,UCCB_MD_MINSPEED_REW,UCCB_MD_MAXSPEED_REW);
        v1=map(vy,0,-My,UCCB_MD_MINSPEED_REW,g_md_maxspeed_rew);
      }
    }
  
  /*
    Serial.print(My);
    Serial.print(" ");
    Serial.print(g_md_maxspeed_fwd);
    Serial.print(" ");
    Serial.println(v1);
  */
    
    if(vx > 0) {
      if(vxn <= g_fs_DC_RIGHT) {
        v2=((long)v1*(g_fs_DC_RIGHT-vxn))/g_fs_DC_RIGHT;
        if(v2 > 0) {
          v2=map(v2,0,v1,UCCB_MD_MINSPEED_FWD,v1);
        } else if(v2 < 0) {
          v2=map(v2,0,v1,UCCB_MD_MINSPEED_REW,v1);
        } else {
          v2=0;
        }
      } else {
        v2=-((long)v1*(vxn-g_fs_DC_RIGHT))/g_fs_DC_RIGHT;
        if(v2 > 0) {
          v2=map(v2,0,abs(v1),UCCB_MD_MINSPEED_FWD,abs(v1));
        } else if(v2 < 0) {
          v2=map(v2,0,-abs(v1),UCCB_MD_MINSPEED_REW,-abs(v1));
        } else {
        	v2=0;
        }
      }
    } else if(vx < 0) {
      if(vxn <= g_fs_DC_LEFT) {
        v2=((long)v1*(g_fs_DC_LEFT-vxn))/g_fs_DC_LEFT;
        if(v2 > 0) {
          v2=map(v2,0,v1,UCCB_MD_MINSPEED_FWD,v1);
        } else if(v2 < 0) {
          v2=map(v2,0,v1,UCCB_MD_MINSPEED_REW,v1);
        } else {
        	v2=0;
        }
      } else {
        v2=-((long)v1*(vxn-g_fs_DC_LEFT))/g_fs_DC_LEFT;
        if(v2 > 0) {
          v2=map(v2,0,abs(v1),UCCB_MD_MINSPEED_FWD,abs(v1));
        } else if(v2 < 0) {
          v2=map(v2,0,-abs(v1),UCCB_MD_MINSPEED_REW,-abs(v1));
        } else {
        	v2=0;
        }
      }
    } else {
      v2=v1;
    }
  }
    
  if(vx > 0) {
    if(vy == 0) {
       *m1s=-v1;
       *m2s=v1;
    } else {
      *m1s=v2;
      *m2s=v1;
    }
  } else {
    if(vy == 0) {
      *m1s=v1;
      *m2s=-v1;
    } else {
      *m1s=v1;
      *m2s=v2;
    }
  }

/*  
  if(*m1s > 0) {
    *m1s=map(*m1s,UCCB_MD_MINSPEED_FWD,UCCB_MD_MAXSPEED_FWD,UCCB_MD_M1_MINSPEED_FWD,UCCB_MD_M1_MAXSPEED_FWD);
  } else if(*m1s < 0) {
    *m1s=map(*m1s,UCCB_MD_MINSPEED_REW,UCCB_MD_MAXSPEED_REW,UCCB_MD_M1_MINSPEED_REW,UCCB_MD_M1_MAXSPEED_REW);
  }
*/  
  
  if(vx > 0) {
//    *rdd=(UCCB_RDD_MAXPOS*vx)/g_fs_x_RIGHT;
    rd0=(UCCB_RDD_MAXPOS*vx)/g_fs_x_RIGHT;
    *rdd=iscale(0,UCCB_RDD_MAXPOS,0,UCCB_RDD_MAXPOS,rd0,g_sh1_rotcurve);
  } else if(vx < 0) {
//    *rdd=(UCCB_RDD_MAXPOS*vx)/g_fs_x_LEFT;
    rd0=(UCCB_RDD_MAXPOS*vx)/g_fs_x_LEFT;
    *rdd=-iscale(0,UCCB_RDD_MAXPOS,0,UCCB_RDD_MAXPOS,-rd0,g_sh1_rotcurve);
  } else {
    rd0=0;
    *rdd=0;
  }
  
/*  
  Serial.print("rd ");
  Serial.print(rd0);
  Serial.print(" ");
  Serial.println(*rdd);
*/  
  
  return(0); 
}

int eval_ts_pos(int *tsx, int *tsy)
{
  int vx,vy,vxn,vyn;
  
  vx=constrain(g_tsX,g_ts_minX,g_ts_maxX);
  vy=constrain(g_tsY,g_ts_minY,g_ts_maxY);
  
  vx=g_ts_cenX-vx;
  vy=g_ts_cenY-vy;
  vxn=abs(vx);
  vyn=abs(vy);

  if(vxn <= UCCB_TS_CENGAP) {
    vx=0;
    vxn=0;
  }
  if(vyn <= UCCB_TS_CENGAP) {
    vy=0;
    vyn=0;
  }
  
  if(vx > 0) {
    *tsx=(100L*vx)/g_ts_x_RIGHT;
  } else if(vx < 0) {
    *tsx=(100L*vx)/g_ts_x_LEFT;
  } else {
    *tsx=0;
  }
  if(vy > 0) {
    *tsy=(100L*vy)/g_ts_y_FWD;
  } else if(vy < 0) {
    *tsy=(100L*vy)/g_ts_y_REW;
  } else {
    *tsy=0;
  }
  
  return(0);  
}

int eval_joy_getpos(int *vx, int *vy, int *vz)
{
  *vx=constrain(g_fsX,g_fs_minX,g_fs_maxX);
  *vy=constrain(g_fsY,g_fs_minY,g_fs_maxY);
  *vz=constrain(g_fsZ,g_fs_minZ,g_fs_maxZ);
  
  *vx=g_fs_cenX-*vx;
  *vy=g_fs_cenY-*vy;
  *vz=g_fs_cenZ-*vz;

  if(abs(*vx) <= UCCB_FS_CENGAP) {
    *vx=0;
  }
  if(abs(*vy) <= UCCB_FS_CENGAP) {
    *vy=0;
  }
  if(abs(*vz) <= UCCB_FS_CENGAP) {
    *vz=0;
  }
  
  return(0);
}

