#include "SoundCard.h"

//namespace Robot_v1
//{
	SoundCard::SoundCard()
	{
		ao_initialize();
		driver = ao_default_driver_id();
		mpg123_init();
	}
	
	void SoundCard::Open(char *path)
	{
		long rate;
		int encoding, channels;
		ao_sample_format format;
		int err;

		/* open the file and get the decoding format */
		mh = mpg123_new(NULL, &err);
		mpg123_open(mh, path);
		mpg123_getformat(mh, &rate, &channels, &encoding);
		buffer_size = mpg123_outblock(mh);
		buffer = (unsigned char*)malloc(buffer_size * sizeof(unsigned char));

		/* set the output format and open the output device */
		format.bits = mpg123_encsize(encoding) << 3;
		format.rate = rate;
		format.channels = channels;
		format.byte_format = AO_FMT_NATIVE;
		format.matrix = 0;
		dev = ao_open_live(driver, &format, NULL);
	}

	void SoundCard::Close()
	{
		free(buffer);
		ao_close(dev);
		mpg123_close(mh);
		mpg123_delete(mh);
	}

	int SoundCard::Play()
	{
		size_t done;
		int retval;		
		
		/* decode and play */
		retval = mpg123_read(mh, buffer, buffer_size, &done);
		if (done > 0) ao_play(dev, (char*)buffer, done);
		else          Close();

		return retval;
	}

	SoundCard::~SoundCard()
	{
		mpg123_exit();
		ao_shutdown();
	}
//}