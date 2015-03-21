#include "encryption.h"
#include "global.h"
#include "md5.h"

static void MDString PROTO_LIST ((unsigned char *));


#define MD5_CTX MD5_CTX
#define MDInit MD5Init
#define MDUpdate MD5Update
#define MDFinal MD5Final

static unsigned char c[21] = "cdhxkj204";
unsigned char digest[16];
static void MDString (unsigned char *string)
{
  MD5_CTX context;  
  unsigned int len = 20;
  MDInit (&context);
  MDUpdate (&context, string, len);
  MDFinal (digest, &context);
}

void Encryption(INT8U *pucSrcBuffer,INT8U *pucDestBuffer)
{
	INT8U i;
	for(i = 0; i<11; i++)
        {
          c[i+9] = pucSrcBuffer[i];
        }
        MDString(c);
	for(i = 0;i < 4;i ++)
	{
		pucDestBuffer[i]= digest[i];
	}
                
        i= 0;	
}

INT8U CheckEncry(INT8U *pucSrcBuffer,INT8U *pucDestBuffer)
{
        INT8U i;
	for(i = 0; i<11; i++)
        {
          c[i+9] = pucSrcBuffer[i];
        }
        MDString(c);
	for(i = 0; i < 4; i++)
	{
            if(pucDestBuffer[i]!= digest[i])
            {
              return 0;//²âÊÔ²»ÅÐ¶¨
            }
	}
                
        return 1;	  
}

