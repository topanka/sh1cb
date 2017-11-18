#include <avr/eeprom.h>

#define UCCB_FS_MINX_EEPROM      0
#define UCCB_FS_MAXX_EEPROM      2
#define UCCB_FS_CENX_EEPROM      4
#define UCCB_FS_MINY_EEPROM      6
#define UCCB_FS_MAXY_EEPROM      8
#define UCCB_FS_CENY_EEPROM      10
#define UCCB_FS_MINZ_EEPROM      12
#define UCCB_FS_MAXZ_EEPROM      14
#define UCCB_FS_CENZ_EEPROM      16

#define UCCB_TS_MINX_EEPROM      18
#define UCCB_TS_MAXX_EEPROM      20
#define UCCB_TS_CENX_EEPROM      22
#define UCCB_TS_MINY_EEPROM      24
#define UCCB_TS_MAXY_EEPROM      26
#define UCCB_TS_CENY_EEPROM      28

#define UCCB_AUTOCLB_NUM         43

int uccb_eeprom_read_int(uint16_t addr)
{
  int val;
  
  val=(int)eeprom_read_word((const uint16_t*)addr);
  return(val);
}
int uccb_eeprom_write_int(uint16_t addr, int val)
{
  int r;
 
  r=uccb_eeprom_read_int(addr);
  if(r != val) {
    eeprom_write_word((uint16_t*)addr,(uint16_t)val);
  }
  return(0);
}

int uccb_clb_read_FSMINX(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MINX_EEPROM));
}
int uccb_clb_write_FSMINX(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MINX_EEPROM,val));
}
int uccb_clb_read_FSMAXX(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MAXX_EEPROM));
}
int uccb_clb_write_FSMAXX(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MAXX_EEPROM,val));
}
int uccb_clb_read_FSCENX(void)
{
  int v;
  
  if(g_fs_cenX0 >= 0) v=g_fs_cenX0;
  else v=uccb_eeprom_read_int(UCCB_FS_CENX_EEPROM);
  return(v);
}
int uccb_clb_write_FSCENX(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_CENX_EEPROM,val));
}

int uccb_clb_read_FSMINY(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MINY_EEPROM));
}
int uccb_clb_write_FSMINY(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MINY_EEPROM,val));
}
int uccb_clb_read_FSMAXY(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MAXY_EEPROM));
}
int uccb_clb_write_FSMAXY(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MAXY_EEPROM,val));
}
int uccb_clb_read_FSCENY(void)
{
  int v;
  
  if(g_fs_cenY0 >= 0) v=g_fs_cenY0;
  else v=uccb_eeprom_read_int(UCCB_FS_CENY_EEPROM);
  
  return(v);
}
int uccb_clb_write_FSCENY(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_CENY_EEPROM,val));
}

int uccb_clb_read_FSMINZ(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MINZ_EEPROM));
}
int uccb_clb_write_FSMINZ(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MINZ_EEPROM,val));
}
int uccb_clb_read_FSMAXZ(void)
{
  return(uccb_eeprom_read_int(UCCB_FS_MAXZ_EEPROM));
}
int uccb_clb_write_FSMAXZ(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_MAXZ_EEPROM,val));
}
int uccb_clb_read_FSCENZ(void)
{
  int v;
  
  if(g_fs_cenZ0 >= 0) v=g_fs_cenZ0;
  else v=uccb_eeprom_read_int(UCCB_FS_CENZ_EEPROM);
  
  return(v);
}
int uccb_clb_write_FSCENZ(int val)
{
  return(uccb_eeprom_write_int(UCCB_FS_CENZ_EEPROM,val));
}

int uccb_clb_read_TSMINX(void)
{
  return(uccb_eeprom_read_int(UCCB_TS_MINX_EEPROM));
}
int uccb_clb_write_TSMINX(int val)
{
  uccb_eeprom_write_int(UCCB_TS_MINX_EEPROM,val);
  g_ts_minX=uccb_clb_read_TSMINX();
  return(0);  
}
int uccb_clb_read_TSMAXX(void)
{
  return(uccb_eeprom_read_int(UCCB_TS_MAXX_EEPROM));
}
int uccb_clb_write_TSMAXX(int val)
{
  uccb_eeprom_write_int(UCCB_TS_MAXX_EEPROM,val);
  g_ts_maxX=uccb_clb_read_TSMAXX();
  return(0);
}
int uccb_clb_read_TSCENX(void)
{
  int v;

  if(g_ts_cenX0 >= 0) v=g_ts_cenX0;
  else v=uccb_eeprom_read_int(UCCB_TS_CENX_EEPROM);
  return(v);
}
int uccb_clb_write_TSCENX(int val)
{
  uccb_eeprom_write_int(UCCB_TS_CENX_EEPROM,val);
  g_ts_cenX=uccb_clb_read_TSCENX();
  return(0);
}

int uccb_clb_read_TSMINY(void)
{
  return(uccb_eeprom_read_int(UCCB_TS_MINY_EEPROM));
}
int uccb_clb_write_TSMINY(int val)
{
  uccb_eeprom_write_int(UCCB_TS_MINY_EEPROM,val);
  g_ts_minY=uccb_clb_read_TSMINY();
  return(0);
}
int uccb_clb_read_TSMAXY(void)
{
  return(uccb_eeprom_read_int(UCCB_TS_MAXY_EEPROM));
}
int uccb_clb_write_TSMAXY(int val)
{
  uccb_eeprom_write_int(UCCB_TS_MAXY_EEPROM,val);
  g_ts_maxY=uccb_clb_read_TSMAXY();
  return(0);
}
int uccb_clb_read_TSCENY(void)
{
  int v;
  
  if(g_ts_cenY0 >= 0) v=g_ts_cenY0;
  else v=uccb_eeprom_read_int(UCCB_TS_CENY_EEPROM);
  
  return(v);
}
int uccb_clb_write_TSCENY(int val)
{
  uccb_eeprom_write_int(UCCB_TS_CENY_EEPROM,val);
  g_ts_cenY=uccb_clb_read_TSCENY();
  return(0);
}

void uccb_clb_load(void)
{
  g_fs_minX=uccb_clb_read_FSMINX();
  g_fs_minY=uccb_clb_read_FSMINY();
  g_fs_minZ=uccb_clb_read_FSMINZ();
  g_fs_maxX=uccb_clb_read_FSMAXX();
  g_fs_maxY=uccb_clb_read_FSMAXY();
  g_fs_maxZ=uccb_clb_read_FSMAXZ();
  g_fs_cenX=uccb_clb_read_FSCENX();
  g_fs_cenY=uccb_clb_read_FSCENY();
  g_fs_cenZ=uccb_clb_read_FSCENZ();

  g_ts_minX=uccb_clb_read_TSMINX();
  g_ts_minY=uccb_clb_read_TSMINY();
  g_ts_maxX=uccb_clb_read_TSMAXX();
  g_ts_maxY=uccb_clb_read_TSMAXY();
  g_ts_cenX=uccb_clb_read_TSCENX();
  g_ts_cenY=uccb_clb_read_TSCENY();
}

int uccb_clb_autocenter_TS(void)
{
  int i,mnx=-1,mxx=-1,mny=-1,mxy=-1,x,y;
  int sx=0,sy=0;
  
  for(i=0;i < UCCB_AUTOCLB_NUM;i++) {
    joy_ts_read(&x,&y);
    delay(5);
  }

  for(i=0;i < UCCB_AUTOCLB_NUM;i++) {
    joy_ts_read(&x,&y);
    
    if(mnx < 0) mnx=x;
    else if(x < mnx)mnx=x;
    if(mxx < 0) mxx=x;
    else if(x > mxx)mxx=x;
    
    if(mny < 0) mny=y;
    else if(y < mny)mny=y;
    if(mxy < 0) mxy=y;
    else if(y > mxy)mxy=y;
    
    sx+=x;
    sy+=y;
    
    delay(10);
  }
  x=sx/UCCB_AUTOCLB_NUM;
  y=sy/UCCB_AUTOCLB_NUM;
 
  g_ts_cenX0=x;
  g_ts_cenY0=y;
  g_ts_cenX=x;
  g_ts_cenY=y;
  
  return(0);
}

int uccb_clb_autocenter_FS(void)
{
  int i,mnx=-1,mxx=-1,mny=-1,mxy=-1,mnz=-1,mxz=-1,x,y,z;
  int sx=0,sy=0,sz=0;
  
  for(i=0;i < UCCB_AUTOCLB_NUM;i++) {
    joy_fs_read(&x,&y,&z);
    delay(2);
  }

  for(i=0;i < UCCB_AUTOCLB_NUM;i++) {
    joy_fs_read(&x,&y,&z);
    
    if(mnx < 0) mnx=x;
    else if(x < mnx) mnx=x;
    if(mxx < 0) mxx=x;
    else if(x > mxx) mxx=x;
    
    if(mny < 0) mny=y;
    else if(y < mny) mny=y;
    if(mxy < 0) mxy=y;
    else if(y > mxy) mxy=y;
    
    if(mnz < 0) mnz=z;
    else if(z < mnz) mnz=z;
    if(mxz < 0) mxz=z;
    else if(z > mxz) mxz=z;
    
    sx+=x;
    sy+=y;
    sz+=z;
    
    delay(2);
  }
  x=sx/UCCB_AUTOCLB_NUM;
  y=sy/UCCB_AUTOCLB_NUM;
  z=sz/UCCB_AUTOCLB_NUM;
 
  g_fs_cenX0=x;
  g_fs_cenY0=y;
  g_fs_cenZ0=z;
  g_fs_cenX=x;
  g_fs_cenY=y;
  g_fs_cenZ=z;
  
  return(0);
}

