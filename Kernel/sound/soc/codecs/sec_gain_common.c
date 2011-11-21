
#include <sound/soc.h>

#include "./twl4030.h"
#include "./sec_gain.h"

#define SEC_AUDIO_DEBUG 0

#if SEC_AUDIO_DEBUG
#define P(format,...)\
		printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

void set_codec_gain(struct snd_soc_codec *codec, int mode, int device)
{
	P("mode : %d, device : %d : this function empty", mode, device);		
}

