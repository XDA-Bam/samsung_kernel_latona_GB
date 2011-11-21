#ifndef _SEC_GAIN_H
#define _SEC_GAIN_H

#define ARX_APGA_GAIN_MASK 0xf8 //0x19, 0x1a, 0x1b, 0x1c
#define EAR_CTL_GAIN_MASK 0x30 //0x21
#define PREDL_CTL_GAIN_MASK 0x30 //0x25, 0x26

void set_codec_gain(struct snd_soc_codec *codec, int mode, int device);
void set_codec_gain_init(struct snd_soc_codec *codec);
#endif // _SEC_GAIN_H