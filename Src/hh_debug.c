#include "hh_debug.h"


float debugMaxOutL=0;
float debugMaxOutR=0;
void debugSetMaxOut(int id,float out)
{
  if(id==0)
  {
    debugMaxOutL=out;
  }
  else
  {
    debugMaxOutR=out;
  }
}
float debugGetMaxOut(int id)
{
  if(id==0)
  {
    return debugMaxOutL;
  }
  else
  {
    return debugMaxOutR;
  }
}


float debugLimitL=0;
float debugLimitR=0;
void debugSetLimit(int id,float limit)
{
  if(id==0)
    debugLimitL=limit;
  else 
    debugLimitR=limit;
}

float debugGetLimit(int id)
{
  if(id==0)
    return debugLimitL;
  else
    return debugLimitR;
}

float debugCurOutL=0;
float debugCurOutR=0;
void debugSetCurOut(int id,float out)
{
  if(id==0)
  {
    debugCurOutL=out;
  }
  else
  {
    debugCurOutR=out;
  }
}
float debugGetCurOut(int id)
{
  if(id==0)
  {
    return debugCurOutL;
  }
  else
  {
    return debugCurOutR;
  }
}
