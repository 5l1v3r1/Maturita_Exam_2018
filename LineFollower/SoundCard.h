#pragma once

#include <ao/ao.h>
#include <mpg123.h>

//namespace Robot_v1
//{
	class SoundCard
	{
		public:
			SoundCard();
			~SoundCard();
			void Open(char *path);
			void Close();
			int Play();
		private:
			int driver;
			ao_device *dev;
			mpg123_handle *mh;
			size_t buffer_size;
			unsigned char *buffer;
	};


//}
